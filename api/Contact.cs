using System.Diagnostics;

namespace Box2D.API;

public static class ContactAPI
{
    ///<summary> Contact identifier validation. Provides validation for up to 2^32 allocations.</summary>
    public static bool IsValid(ContactID id)
    {
        World world = id.world0;
        if (world == null) return false;
        int contactId = id.index1 - 1;
        if (contactId < 0 || world.contacts.Count <= contactId) return false;
        Contact contact = world.contacts[contactId];
        Debug.Assert(contact.contactId == contactId);
        return id.generation == contact.generation;
    }

    ///<summary> Get the data for a contact. The manifold may have no points if the contact is not touching.</summary>
    public static ContactData GetData(ContactID contactId)
    {
        World world = contactId.world0;
        Contact contact = world.GetContactFullId(contactId);
        ContactSim contactSim = world.GetContactSim(contact);
        Shape shapeA = world.shapes[contact.shapeIdA], shapeB = world.shapes[contact.shapeIdB];
        return new()
        {
            contactId = contactId,
            shapeIdA = new() { index1 = shapeA.id + 1, world0 = contactId.world0, generation = shapeA.generation },
            shapeIdB = new() { index1 = shapeB.id + 1, world0 = contactId.world0, generation = shapeB.generation },
            manifold = contactSim.manifold
        };
    }
}
