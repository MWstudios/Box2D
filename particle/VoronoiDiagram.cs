using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace Box2D.Particle;

public unsafe class VoronoiDiagram
{
    public VoronoiDiagram(int generatorCapacity)
    {
        m_generatorBuffer = new(generatorCapacity);
    }
    public void AddGenerator(Vector2 center, int tag, bool necessary)
    {
        Generator g = new() { center = center, tag = tag, necessary = necessary };
        m_generatorBuffer.Add(g);
    }
    public void Generate(float radius, float margin)
    {
        float inverseRadius = 1 / radius;
        Vector2 lower = new(float.MaxValue, float.MaxValue);
        Vector2 upper = new(float.MinValue, float.MinValue);
        foreach (var g in m_generatorBuffer) if (g.necessary)
            {
                lower = Vector2.Min(lower, g.center);
                upper = Vector2.Max(upper, g.center);
            }
        lower.x -= margin; lower.y -= margin;
        upper.x += margin; upper.y += margin;
        m_countX = 1 + (int)(inverseRadius * (upper.x - lower.x));
        m_countY = 1 + (int)(inverseRadius * (upper.y - lower.y));
        m_diagram = new Generator[m_countX * m_countY];
        Queue<VoronoiDiagramTask> queue = new(4 * m_countX * m_countY);
        foreach (var g in m_generatorBuffer)
        {
            g.center = inverseRadius * (g.center - lower);
            int x = (int)g.center.x, y = (int)g.center.y;
            if (x >= 0 && y >= 0 && x < m_countX && y < m_countY)
                queue.Enqueue(new(x, y, x + y * m_countX, g));
        }
        while (queue.Count > 0)
        {
            var q = queue.Dequeue();
            int x = q.m_x, y = q.m_y, i = q.m_i;
            Generator g = q.m_generator;
            if (m_diagram[i] == null)
            {
                m_diagram[i] = g;
                if (x > 0) queue.Enqueue(new(x - 1, y, i - 1, g));
                if (y > 0) queue.Enqueue(new(x, y - 1, i - m_countX, g));
                if (x < m_countX - 1) queue.Enqueue(new(x + 1, y, i + 1, g));
                if (y < m_countY - 1) queue.Enqueue(new(x, y + 1, i + m_countX, g));
            }
        }
        for (int y = 0; y < m_countY; y++)
        {
            for (int x = 0; x < m_countX-1; x++)
            {
                int i = x + y * m_countX;
                Generator a = m_diagram[i];
                Generator b = m_diagram[i + 1];
                if (a != b)
                {
                    queue.Enqueue(new(x, y, i, b));
                    queue.Enqueue(new(x + 1, y, i + 1, a));
                }
            }
        }
        for (int y = 0; y < m_countY - 1; y++)
        {
            for (int x = 0; x < m_countX; x++)
            {
                int i = x + y * m_countX;
                Generator a = m_diagram[i];
                Generator b = m_diagram[i + m_countX];
                if (a != b)
                {
                    queue.Enqueue(new(x, y, i, b));
                    queue.Enqueue(new(x, y + 1, i + m_countX, a));
                }
            }
        }
        while (queue.Count > 0)
        {
            var task = queue.Dequeue();
            int x = task.m_x, y = task.m_y, i = task.m_i;
            Generator k = task.m_generator;
            Generator a = m_diagram[i];
            Generator b = k;
            if (a != b)
            {
                if ((a.center - new Vector2(x, y)).LengthSquared() > (b.center - new Vector2(x, y)).LengthSquared())
                {
                    m_diagram[i] = b;
                    if (x > 0) queue.Enqueue(new(x - 1, y, i - 1, b));
                    if (y > 0) queue.Enqueue(new(x, y - 1, i - m_countX, b));
                    if (x < m_countX - 1) queue.Enqueue(new(x + 1, y, i + 1, b));
                    if (y < m_countY - 1) queue.Enqueue(new(x, y + 1, i + m_countX, b));
                }
            }
        }
    }
    public delegate void NodeCallback(int a, int b, int c);
    public void GetNodes(NodeCallback callback)
    {
        for (int y = 0; y < m_countY-1; y++)
        {
            for (int x = 0; x < m_countX-1; x++)
            {
                int i = x + y * m_countX;
                Generator a = m_diagram[i], b = m_diagram[i + 1], c = m_diagram[i + m_countX], d = m_diagram[i + m_countX + 1];
                if (b != c)
                {
                    if (a != b && a != c && (a.necessary || b.necessary || c.necessary)) callback(a.tag, b.tag, c.tag);
                    if (d != b && d != c && (b.necessary || d.necessary || c.necessary)) callback(b.tag, d.tag, c.tag);
                }
            }
        }
    }
    [DebuggerDisplay("{center}; {tag}; {necessary}")] class Generator
    {
        public Vector2 center; public int tag; public bool necessary;
        //public static bool operator ==(Generator g1, Generator g2) => g1.center == g2.center && g1.tag == g2.tag && g1.necessary == g2.necessary;
        //public static bool operator !=(Generator g1, Generator g2) => g1.center != g2.center || g1.tag != g2.tag || g1.necessary != g2.necessary;
        //public override bool Equals(object obj) => obj is Generator g && g == this;
        public override int GetHashCode() => HashCode.Combine(center, tag, necessary);
    }
    struct VoronoiDiagramTask
    {
        public int m_x, m_y, m_i;
        public Generator m_generator;
        public VoronoiDiagramTask(int x, int y, int i, Generator g)
        { m_x = x; m_y = y; m_i = i; m_generator = g; }
    }
    Generator[] m_diagram;
    HashSet<Generator> m_generatorBuffer;
    int m_countX = 0, m_countY = 0;
}
