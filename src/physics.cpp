#include "physics.h"

glm::dmat3 setSkewSymmetric(glm::dmat3 m1, const glm::dvec3 vector)
{
    m1[0][0] = m1[1][1] = m1[2][2] = 0;
    m1[0][1] = -vector.z;
    m1[0][2] = vector.y;
    m1[1][0] = vector.z;
    m1[1][2] = -vector.x;
    m1[2][0] = -vector.y;
    m1[2][1] = vector.x;
    return m1;
}

glm::dvec3 getAxisVector(glm::dmat4 m, int i)
{
    return glm::dvec3(m[0][i], m[1][i], m[2][i]);
}

/*
 * Definition of the sleep epsilon extern.
 */
double sleepEpsilon = 0.3;

glm::dmat3 setBlockInertiaTensor(glm::dmat3 m, const glm::dvec3 &halfSizes, double mass)
{
    glm::dvec3 squares = halfSizes * halfSizes;
    m = setInertiaTensorCoeffs(m, 0.3f*mass*(squares.y + squares.z),
        0.3f*mass*(squares.x + squares.z),
        0.3f*mass*(squares.x + squares.y));

    return m;
}

glm::dmat3 setInertiaTensorCoeffs(glm::dmat3 m, double ix, double iy, double iz,
    double ixy, double ixz, double iyz)
{
    m[0][0] = ix;
    m[0][1] = m[1][0] = -ixy;
    m[0][2] = m[2][0] = -ixz;
    m[1][1] = iy;
    m[1][2] = m[2][1] = -iyz;
    m[2][2] = iz;
    return m;
}

void RigidBody::integrate(double duration)
{
    if (!isAwake) return;

    // Calculate linear acceleration from force inputs.
    lastFrameAcceleration = acceleration;
    lastFrameAcceleration += forceAccum * inverseMass;

    // Calculate angular acceleration from torque inputs.
    glm::dvec3 angularAcceleration =  torqueAccum * inverseInertiaTensorWorld;

    // Adjust velocities
    // Update linear velocity from both acceleration and impulse.
    velocity += lastFrameAcceleration * duration;

    // Update angular velocity from both acceleration and impulse.
    rotation += angularAcceleration * duration;

    // Impose drag.
    velocity *= pow(linearDamping, duration);
    rotation *= pow(angularDamping, duration);

    // Adjust positions
    // Update linear position.
    position += velocity * duration;

    // Update angular position.
    orientation += glm::dquat(0, rotation * duration) * orientation * 0.5;

    // Normalise the orientation, and update the matrices with the new
    // position and orientation
    orientation = normalize(orientation);

    // Calculate the transform matrix for the body.
    transformMatrix = (glm::dmat4)orientation;
    transformMatrix[3] = glm::dvec4(position, 1.0);

    // Calculate the inertiaTensor in world space.
    inverseInertiaTensorWorld = glm::dmat3(transformMatrix) * inverseInertiaTensor * glm::dmat3(glm::inverse(transformMatrix));

    // Clear accumulators.
    forceAccum = glm::dvec3(0.0, 0.0, 0.0);
    torqueAccum = glm::dvec3(0.0, 0.0, 0.0);

    // Update the kinetic energy store, and possibly put the body to
    // sleep.
    if (canSleep) {
        double currentMotion = dot(velocity, velocity) +
            dot(rotation, rotation);

        double bias = pow(0.5, duration);
        motion = bias*motion + (1-bias)*currentMotion;

        if (motion < sleepEpsilon) setAwake(false);
        else if (motion > 10 * sleepEpsilon) motion = 10 * sleepEpsilon;
    }
}

void RigidBody::setMass(const double mass)
{
    assert(mass != 0);
    RigidBody::inverseMass = ((double)1.0)/mass;
}

void RigidBody::setAwake(const bool awake)
{
    if (awake) {
        isAwake= true;

        // Add a bit of motion to avoid it falling asleep immediately.
        motion = sleepEpsilon*2.0f;
    } else {
        isAwake = false;
        velocity = glm::dvec3(0.0, 0.0, 0.0);
        rotation = glm::dvec3(0.0, 0.0, 0.0);
    }
}

void RigidBody::setCanSleep(const bool canSleep)
{
    RigidBody::canSleep = canSleep;

    if (!canSleep && !isAwake) setAwake();
}

//std::vector<CollisionBox*> CollisionBox::allBoxes = {};
CollisionBox* CollisionBox::boxData[256] = {};
unsigned int CollisionBox::boxCount = 0;

CollisionBox::CollisionBox(double x, double y, double z, glm::vec3 orientation, double gravity, double rx, double ry, double rz, double sx, double sy, double sz)
{
    body = new RigidBody;
    setState(x,y,z, orientation, gravity, rx, ry, rz, sx, sy, sz);
    boxID = CollisionBox::boxCount;
    CollisionBox::boxData[boxID] = this;
    CollisionBox::boxCount++;
}

/** Sets the box to a specific location. */
void CollisionBox::setState(double x, double y, double z,  glm::vec3 orientation, double gravity, double rx, double ry, double rz, double sx, double sy, double sz)
{
    body->position = glm::dvec3(x, y, z);
    body->orientation = glm::quat(orientation);
    body->velocity = glm::dvec3(0,0,0);
    body->rotation = glm::dvec3(rx, ry, rz);
    halfSize = glm::dvec3(sx, sy, sz);

    double mass = halfSize.x * halfSize.y * halfSize.z * 8.0f;
    body->setMass(mass);

    glm::dmat3 tensor = glm::dmat3(1.0);
    tensor = setBlockInertiaTensor(tensor, halfSize, mass);
    body->inverseInertiaTensor = glm::inverse(tensor);
    body->linearDamping = 0.95f;
    body->angularDamping = 0.8f;
    body->forceAccum = glm::dvec3(0.0, 0.0, 0.0);
    body->torqueAccum = glm::dvec3(0.0, 0.0, 0.0);
    body->acceleration = glm::dvec3(0,gravity,0);

    body->setCanSleep(false);
    body->isAwake = true;

    // position and orientation
    body->orientation = normalize(body->orientation);

    // Calculate the transform matrix for the body.
    body->transformMatrix = (glm::dmat4)body->orientation;
    body->transformMatrix[3] = glm::dvec4(body->position, 1.0);
    // Calculate the inertiaTensor in world space.
    body->inverseInertiaTensorWorld = glm::dmat3(body->transformMatrix) * body->inverseInertiaTensor * glm::dmat3(glm::inverse(body->transformMatrix));

    transform = glm::transpose(body->transformMatrix) * offset;
}

glm::mat4 CollisionBox::getModelMatrix() {
    return CollisionBox::boxData[boxID]->modelMatrix;
}
void CollisionBox::setModelMatrix(glm::mat4 m) {
    CollisionBox::boxData[boxID]->modelMatrix = m;
}

void CollisionBox::calculateInternals()
{
    transform = glm::transpose(body->transformMatrix) * offset;
}


static inline double transformToAxis(
    const CollisionBox &box,
    const glm::dvec3 &axis
    )
{
    return
        box.halfSize.x * fabs(dot(axis, box.getAxis(0))) +
        box.halfSize.y * fabs(dot(axis, box.getAxis(1))) +
        box.halfSize.z * fabs(dot(axis, box.getAxis(2)));
}

/**
 * This function checks if the two boxes overlap
 * along the given axis. The final parameter toCentre
 * is used to pass in the vector between the boxes centre
 * points, to avoid having to recalculate it each time.
 */
static inline bool overlapOnAxis(
    const CollisionBox &one,
    const CollisionBox &two,
    const glm::dvec3 &axis,
    const glm::dvec3 &toCentre
    )
{
    // Project the half-size of one onto axis
    double oneProject = transformToAxis(one, axis);
    double twoProject = transformToAxis(two, axis);

    // Project this onto the axis
    double distance = fabs(dot(toCentre, axis));

    // Check for overlap
    return (distance < oneProject + twoProject);
}

// This preprocessor definition is only used as a convenience
// in the boxAndBox intersection  method.
#define TEST_OVERLAP(axis) overlapOnAxis(one, two, (axis), toCentre)

bool IntersectionTests::boxAndBox(
    const CollisionBox &one,
    const CollisionBox &two
    )
{
    // Find the vector between the two centres
    glm::dvec3 toCentre = two.getAxis(3) - one.getAxis(3);

    return (
        // Check on box one's axes first
        TEST_OVERLAP(one.getAxis(0)) &&
        TEST_OVERLAP(one.getAxis(1)) &&
        TEST_OVERLAP(one.getAxis(2)) &&

        // And on two's
        TEST_OVERLAP(two.getAxis(0)) &&
        TEST_OVERLAP(two.getAxis(1)) &&
        TEST_OVERLAP(two.getAxis(2)) &&

        // Now on the cross products
        TEST_OVERLAP(cross(one.getAxis(0), two.getAxis(0))) &&
        TEST_OVERLAP(cross(one.getAxis(0), two.getAxis(1))) &&
        TEST_OVERLAP(cross(one.getAxis(0), two.getAxis(2))) &&
        TEST_OVERLAP(cross(one.getAxis(1), two.getAxis(0))) &&
        TEST_OVERLAP(cross(one.getAxis(1), two.getAxis(1))) &&
        TEST_OVERLAP(cross(one.getAxis(1), two.getAxis(2))) &&
        TEST_OVERLAP(cross(one.getAxis(2), two.getAxis(0))) &&
        TEST_OVERLAP(cross(one.getAxis(2), two.getAxis(1))) &&
        TEST_OVERLAP(cross(one.getAxis(2), two.getAxis(2)))
    );
}
#undef TEST_OVERLAP


/*
 * This function checks if the two boxes overlap
 * along the given axis, returning the ammount of overlap.
 * The final parameter toCentre
 * is used to pass in the vector between the boxes centre
 * points, to avoid having to recalculate it each time.
 */
static inline double penetrationOnAxis(
    const CollisionBox &one,
    const CollisionBox &two,
    const glm::dvec3 &axis,
    const glm::dvec3 &toCentre
    )
{
    // Project the half-size of one onto axis
    double oneProject = transformToAxis(one, axis);
    double twoProject = transformToAxis(two, axis);

    // Project this onto the axis
    double distance = fabs(dot(toCentre, axis));

    // Return the overlap (i.e. positive indicates
    // overlap, negative indicates separation).
    return oneProject + twoProject - distance;
}


static inline bool tryAxis(
    const CollisionBox &one,
    const CollisionBox &two,
    glm::dvec3 axis,
    const glm::dvec3& toCentre,
    unsigned index,

    // These values may be updated
    double& smallestPenetration,
    unsigned &smallestCase
    )
{
    // Make sure we have a normalized axis, and don't check almost parallel axes
    if (length2(axis) < 0.0001) return true;
    axis = normalize(axis);

    double penetration = penetrationOnAxis(one, two, axis, toCentre);

    if (penetration < 0) return false;
    if (penetration < smallestPenetration) {
        smallestPenetration = penetration;
        smallestCase = index;
    }
    return true;
}

void fillPointFaceBoxBox(
    const CollisionBox &one,
    const CollisionBox &two,
    const glm::dvec3 &toCentre,
    CollisionData *data,
    unsigned best,
    double pen
    )
{
    // This method is called when we know that a vertex from
    // box two is in contact with box one.

    Contact* contact = data->contacts;

    // We know which axis the collision is on (i.e. best),
    // but we need to work out which of the two faces on
    // this axis.
    glm::dvec3 normal = one.getAxis(best);
    if (dot(one.getAxis(best), toCentre) > 0)
    {
        normal = normal * -1.0;
    }

    // Work out which vertex of box two we're colliding with.
    // Using toCentre doesn't work!
    glm::dvec3 vertex = two.halfSize;
    if (dot(two.getAxis(0), normal) < 0) vertex.x = -vertex.x;
    if (dot(two.getAxis(1), normal) < 0) vertex.y = -vertex.y;
    if (dot(two.getAxis(2), normal) < 0) vertex.z = -vertex.z;

    // Create the contact data
    contact->contactNormal = normal;
    contact->penetration = pen;
    glm::dvec4 tmp = glm::dvec4(vertex.x, vertex.y, vertex.z, 1.0) * two.getTransform();
    glm::dvec3 v = glm::dvec3(tmp.x, tmp.y, tmp.z);

    contact->contactPoint = v;
    contact->setBodyData(one.body, two.body,
        data->friction, data->restitution);
}

static inline glm::dvec3 contactPoint(
    const glm::dvec3 &pOne,
    const glm::dvec3 &dOne,
    double oneSize,
    const glm::dvec3 &pTwo,
    const glm::dvec3 &dTwo,
    double twoSize,

    // If this is true, and the contact point is outside
    // the edge (in the case of an edge-face contact) then
    // we use one's midpoint, otherwise we use two's.
    bool useOne)
{
    glm::dvec3 toSt, cOne, cTwo;
    double dpStaOne, dpStaTwo, dpOneTwo, smOne, smTwo;
    double denom, mua, mub;

    smOne = length2(dOne);
    smTwo = length2(dTwo);
    dpOneTwo = dot(dTwo, dOne);

    toSt = pOne - pTwo;
    dpStaOne = dot(dOne, toSt);
    dpStaTwo = dot(dTwo, toSt);

    denom = smOne * smTwo - dpOneTwo * dpOneTwo;

    // Zero denominator indicates parrallel lines
    if (fabs(denom) < 0.0001f) {
        return useOne?pOne:pTwo;
    }

    mua = (dpOneTwo * dpStaTwo - smTwo * dpStaOne) / denom;
    mub = (smOne * dpStaTwo - dpOneTwo * dpStaOne) / denom;

    // If either of the edges has the nearest point out
    // of bounds, then the edges aren't crossed, we have
    // an edge-face contact. Our point is on the edge, which
    // we know from the useOne parameter.
    if (mua > oneSize ||
        mua < -oneSize ||
        mub > twoSize ||
        mub < -twoSize)
    {
        return useOne?pOne:pTwo;
    }
    else
    {
        cOne = pOne + dOne * mua;
        cTwo = pTwo + dTwo * mub;

        return cOne * 0.5 + cTwo * 0.5;
    }
}

// This preprocessor definition is only used as a convenience
// in the boxAndBox contact generation method.
#define CHECK_OVERLAP(axis, index) \
    if (!tryAxis(one, two, (axis), toCentre, (index), pen, best)) return 0;

unsigned CollisionDetector::boxAndBox(
    const CollisionBox &one,
    const CollisionBox &two,
    CollisionData *data
    )
{
    //if (!IntersectionTests::boxAndBox(one, two)) return 0;

    // Find the vector between the two centres
    glm::dvec3 toCentre = two.getAxis(3) - one.getAxis(3);

    // We start assuming there is no contact
    double pen = FLT_MAX;
    unsigned best = 0xffffff;

    // Now we check each axes, returning if it gives us
    // a separating axis, and keeping track of the axis with
    // the smallest penetration otherwise.
    CHECK_OVERLAP(one.getAxis(0), 0);
    CHECK_OVERLAP(one.getAxis(1), 1);
    CHECK_OVERLAP(one.getAxis(2), 2);

    CHECK_OVERLAP(two.getAxis(0), 3);
    CHECK_OVERLAP(two.getAxis(1), 4);
    CHECK_OVERLAP(two.getAxis(2), 5);

    // Store the best axis-major, in case we run into almost
    // parallel edge collisions later
    unsigned bestSingleAxis = best;

    CHECK_OVERLAP(cross(one.getAxis(0), two.getAxis(0)), 6);
    CHECK_OVERLAP(cross(one.getAxis(0), two.getAxis(1)), 7);
    CHECK_OVERLAP(cross(one.getAxis(0), two.getAxis(2)), 8);
    CHECK_OVERLAP(cross(one.getAxis(1), two.getAxis(0)), 9);
    CHECK_OVERLAP(cross(one.getAxis(1), two.getAxis(1)), 10);
    CHECK_OVERLAP(cross(one.getAxis(1), two.getAxis(2)), 11);
    CHECK_OVERLAP(cross(one.getAxis(2), two.getAxis(0)), 12);
    CHECK_OVERLAP(cross(one.getAxis(2), two.getAxis(1)), 13);
    CHECK_OVERLAP(cross(one.getAxis(2), two.getAxis(2)), 14);

    // Make sure we've got a result.
    assert(best != 0xffffff);

    // We now know there's a collision, and we know which
    // of the axes gave the smallest penetration. We now
    // can deal with it in different ways depending on
    // the case.
    if (best < 3)
    {
        // We've got a vertex of box two on a face of box one.
        fillPointFaceBoxBox(one, two, toCentre, data, best, pen);
        data->addContacts(1);
        return 1;
    }
    else if (best < 6)
    {
        // We've got a vertex of box one on a face of box two.
        // We use the same algorithm as above, but swap around
        // one and two (and therefore also the vector between their
        // centres).
        fillPointFaceBoxBox(two, one, toCentre*-1.0, data, best-3, pen);
        data->addContacts(1);
        return 1;
    }
    else
    {
        // We've got an edge-edge contact. Find out which axes
        best -= 6;
        unsigned oneAxisIndex = best / 3;
        unsigned twoAxisIndex = best % 3;
        glm::dvec3 oneAxis = one.getAxis(oneAxisIndex);
        glm::dvec3 twoAxis = two.getAxis(twoAxisIndex);
        glm::dvec3 axis = cross(oneAxis, twoAxis);
        axis = normalize(axis);

        // The axis should point from box one to box two.
        if (dot(axis, toCentre) > 0) {
            axis = axis * -1.0;
        }
        // We have the axes, but not the edges: each axis has 4 edges parallel
        // to it, we need to find which of the 4 for each object. We do
        // that by finding the point in the centre of the edge. We know
        // its component in the direction of the box's collision axis is zero
        // (its a mid-point) and we determine which of the extremes in each
        // of the other axes is closest.
        glm::dvec3 ptOnOneEdge = one.halfSize;
        glm::dvec3 ptOnTwoEdge = two.halfSize;
        for (unsigned i = 0; i < 3; i++)
        {
            if (i == oneAxisIndex) ptOnOneEdge[i] = 0;
            else if (dot(one.getAxis(i), axis) > 0) ptOnOneEdge[i] = -ptOnOneEdge[i];

            if (i == twoAxisIndex) ptOnTwoEdge[i] = 0;
            else if (dot(two.getAxis(i), axis) < 0) ptOnTwoEdge[i] = -ptOnTwoEdge[i];
        }
        // Move them into world coordinates (they are already oriented
        // correctly, since they have been derived from the axes).

        glm::dvec4 tmp = glm::dvec4(ptOnOneEdge.x, ptOnOneEdge.y, ptOnOneEdge.z, 1.0) * one.transform;
        glm::dvec3 v(tmp.x, tmp.y, tmp.z);

        ptOnOneEdge = v;

        tmp = glm::dvec4(ptOnTwoEdge.x, ptOnTwoEdge.y, ptOnTwoEdge.z, 1.0) * two.transform;
        v = glm::dvec3(tmp.x, tmp.y, tmp.z);

        ptOnTwoEdge = v;

        // So we have a point and a direction for the colliding edges.
        // We need to find out point of closest approach of the two
        // line-segments.
        glm::dvec3 vertex = contactPoint(
            ptOnOneEdge, oneAxis, one.halfSize[oneAxisIndex],
            ptOnTwoEdge, twoAxis, two.halfSize[twoAxisIndex],
            bestSingleAxis > 2
            );

        // We can fill the contact.
        Contact* contact = data->contacts;

        contact->penetration = pen;
        contact->contactNormal = axis;
        contact->contactPoint = vertex;
        contact->setBodyData(one.body, two.body,
            data->friction, data->restitution);
        data->addContacts(1);
        return 1;
    }
    return 0;
}
#undef CHECK_OVERLAP

// Contact implementation

void Contact::setBodyData(RigidBody* one, RigidBody *two,
                          double friction, double restitution)
{
    Contact::body[0] = one;
    Contact::body[1] = two;
    Contact::friction = friction;
    Contact::restitution = restitution;
}

void Contact::matchAwakeState()
{
    // Collisions with the world never cause a body to wake up.
    if (!body[1]) return;

    bool body0awake = body[0]->isAwake;
    bool body1awake = body[1]->isAwake;

    // Wake up only the sleeping one
    if (body0awake ^ body1awake) {
        if (body0awake) {
            body[1]->isAwake = true;
        }
        else {
            body[0]->isAwake = true;
        }
    }
}

/*
 * Swaps the bodies in the current contact, so body 0 is at body 1 and
 * vice versa. This also changes the direction of the contact normal,
 * but doesn't update any calculated internal mat. If you are calling
 * this method manually, then call calculateInternals afterwards to
 * make sure the internal mat is up to date.
 */
void Contact::swapBodies()
{
    contactNormal *= -1;

    RigidBody *temp = body[0];
    body[0] = body[1];
    body[1] = temp;
}

/*
 * Constructs an arbitrary orthonormal basis for the contact.  This is
 * stored as a 3x3 matrix, where each vector is a column (in other
 * words the matrix transforms contact space into world space). The x
 * direction is generated from the contact normal, and the y and z
 * directionss are set so they are at right angles to it.
 */
inline
void Contact::calculateContactBasis()
{
    glm::dvec3 contactTangent[2];

    // Check whether the Z-axis is nearer to the X or Y axis
    if (fabs(contactNormal.x) > fabs(contactNormal.y))
    {
        // Scaling factor to ensure the results are normalised
        const double s = (double)1.0f/sqrt(contactNormal.z*contactNormal.z +
            contactNormal.x*contactNormal.x);

        // The new X-axis is at right angles to the world Y-axis
        contactTangent[0].x = contactNormal.z*s;
        contactTangent[0].y = 0;
        contactTangent[0].z = -contactNormal.x*s;

        // The new Y-axis is at right angles to the new X- and Z- axes
        contactTangent[1].x = contactNormal.y*contactTangent[0].x;
        contactTangent[1].y = contactNormal.z*contactTangent[0].x -
            contactNormal.x*contactTangent[0].z;
        contactTangent[1].z = -contactNormal.y*contactTangent[0].x;
    }
    else
    {
        // Scaling factor to ensure the results are normalised
        const double s = (double)1.0/sqrt(contactNormal.z*contactNormal.z +
            contactNormal.y*contactNormal.y);

        // The new X-axis is at right angles to the world X-axis
        contactTangent[0].x = 0;
        contactTangent[0].y = -contactNormal.z*s;
        contactTangent[0].z = contactNormal.y*s;

        // The new Y-axis is at right angles to the new X- and Z- axes
        contactTangent[1].x = contactNormal.y*contactTangent[0].z -
            contactNormal.z*contactTangent[0].y;
        contactTangent[1].y = -contactNormal.x*contactTangent[0].z;
        contactTangent[1].z = contactNormal.x*contactTangent[0].y;
    }

    // Make a matrix from the three vectors.
    contactToWorld = glm::dmat3(contactNormal, contactTangent[0], contactTangent[1]);
}

glm::dvec3 Contact::calculateLocalVelocity(unsigned bodyIndex, double duration)
{
    RigidBody *thisBody = body[bodyIndex];

    // Work out the velocity of the contact point.
    glm::dvec3 velocity =
        cross(thisBody->rotation, relativeContactPosition[bodyIndex]);
    velocity += thisBody->velocity;

    // Turn the velocity into contact-coordinates.
    glm::dvec3 contactVelocity = glm::transpose(contactToWorld) * velocity;

    // Calculate the ammount of velocity that is due to forces without
    // reactions.
    glm::dvec3 accVelocity = thisBody->lastFrameAcceleration * duration;

    // Calculate the velocity in contact-coordinates.
    accVelocity = glm::transpose(contactToWorld) * accVelocity;

    // We ignore any component of acceleration in the contact normal
    // direction, we are only interested in planar acceleration
    accVelocity.x = 0;

    // Add the planar velocities - if there's enough friction they will
    // be removed during velocity resolution
    contactVelocity += accVelocity;

    // And return it
    return contactVelocity;
}


void Contact::calculateDesiredDeltaVelocity(double duration)
{
    const static double velocityLimit = (double)0.25f;

    // Calculate the acceleration induced velocity accumulated this frame
    double velocityFromAcc = 0;

    if (body[0]->isAwake)
    {
	velocityFromAcc+=
	    dot(body[0]->lastFrameAcceleration * duration, contactNormal);
    }

    if (body[1] && body[1]->isAwake)
    {
        velocityFromAcc -=
            dot(body[1]->lastFrameAcceleration * duration, contactNormal);
    }

    // If the velocity is very slow, limit the restitution
    double thisRestitution = restitution;
    if (fabs(contactVelocity.x) < velocityLimit)
    {
        thisRestitution = (double)0.0f;
    }

    // Combine the bounce velocity with the removed
    // acceleration velocity.
    desiredDeltaVelocity =
        -contactVelocity.x
        -thisRestitution * (contactVelocity.x - velocityFromAcc);
}


void Contact::calculateInternals(double duration)
{
    // Check if the first object is NULL, and swap if it is.
    if (!body[0]) swapBodies();
    assert(body[0]);

    // Calculate an set of axis at the contact point.
    calculateContactBasis();

    // Store the relative position of the contact relative to each body
    relativeContactPosition[0] = contactPoint - body[0]->position;
    if (body[1]) {
        relativeContactPosition[1] = contactPoint - body[1]->position;
    }

    // Find the relative velocity of the bodies at the contact point.
    contactVelocity = calculateLocalVelocity(0, duration);
    if (body[1]) {
        contactVelocity -= calculateLocalVelocity(1, duration);
    }

    // Calculate the desired change in velocity for resolution
    calculateDesiredDeltaVelocity(duration);
}

void Contact::applyVelocityChange(glm::dvec3 velocityChange[2],
                                  glm::dvec3 rotationChange[2])
{
    // Get hold of the inverse mass and inverse inertia tensor, both in
    // world coordinates.
    glm::dmat3 inverseInertiaTensor[2] = {glm::dmat3(1.0), glm::dmat3(1.0)};
    inverseInertiaTensor[0] =  body[0]->inverseInertiaTensorWorld;
    if (body[1])
        inverseInertiaTensor[1] = body[1]->inverseInertiaTensorWorld;

    // We will calculate the impulse for each contact axis
    glm::dvec3 impulseContact;

    if (friction == (double)0.0)
    {
        // Use the short format for frictionless contacts
        impulseContact = calculateFrictionlessImpulse(inverseInertiaTensor);
    }
    else
    {
        // Otherwise we may have impulses that aren't in the direction of the
        // contact, so we need the more complex version.
        impulseContact = calculateFrictionImpulse(inverseInertiaTensor);
    }

    // Convert impulse to world coordinates
    glm::dvec3 impulse = impulseContact * glm::transpose(contactToWorld);

    // Split in the impulse into linear and rotational components
    glm::dvec3 impulsiveTorque = cross(relativeContactPosition[0], impulse);
    rotationChange[0] = impulsiveTorque * inverseInertiaTensor[0];
    velocityChange[0] = glm::dvec3(0.0, 0.0, 0.0);
    velocityChange[0] += impulse * body[0]->inverseMass;

    // Apply the changes
    body[0]->velocity += velocityChange[0];
    body[0]->rotation += rotationChange[0];

    if (body[1])
    {
        // Work out body one's linear and angular changes
        glm::dvec3 impulsiveTorque = cross(impulse, relativeContactPosition[1]);
        rotationChange[1] = impulsiveTorque * inverseInertiaTensor[1];
        velocityChange[1] = glm::dvec3(0.0, 0.0, 0.0);
        velocityChange[1] += impulse * -body[1]->inverseMass;

        // And apply them.
        body[1]->velocity += velocityChange[1];
        body[1]->rotation += rotationChange[1];
    }
}

inline
glm::dvec3 Contact::calculateFrictionlessImpulse(glm::dmat3 * inverseInertiaTensor)
{
    glm::dvec3 impulseContact;

    // Build a vector that shows the change in velocity in
    // world space for a unit impulse in the direction of the contact
    // normal.
    glm::dvec3 deltaVelWorld = cross(relativeContactPosition[0], contactNormal);
    deltaVelWorld = deltaVelWorld * inverseInertiaTensor[0];
    deltaVelWorld = cross(deltaVelWorld, relativeContactPosition[0]);

    // Work out the change in velocity in contact coordiantes.
    double deltaVelocity = dot(deltaVelWorld, contactNormal);

    // Add the linear component of velocity change
    deltaVelocity += body[0]->inverseMass;

    // Check if we need to the second body's mat
    if (body[1])
    {
        // Go through the same transformation sequence again
        glm::dvec3 deltaVelWorld = cross(relativeContactPosition[1], contactNormal);
        deltaVelWorld = deltaVelWorld * inverseInertiaTensor[1];
        deltaVelWorld = cross(deltaVelWorld, relativeContactPosition[1]);

        // Add the change in velocity due to rotation
        deltaVelocity += dot(deltaVelWorld, contactNormal);

        // Add the change in velocity due to linear motion
        deltaVelocity += body[1]->inverseMass;
    }

    // Calculate the required size of the impulse
    impulseContact.x = desiredDeltaVelocity / deltaVelocity;
    impulseContact.y = 0;
    impulseContact.z = 0;
    return impulseContact;
}

inline
glm::dvec3 Contact::calculateFrictionImpulse(glm::dmat3 * inverseInertiaTensor)
{
    glm::dvec3 impulseContact;
    double inverseMass = body[0]->inverseMass;

    // The equivalent of a cross product in matrices is multiplication
    // by a skew symmetric matrix - we build the matrix for converting
    // between linear and angular quantities.
    glm::dmat3 impulseToTorque = glm::dmat3(1.0);
    impulseToTorque = setSkewSymmetric(impulseToTorque, relativeContactPosition[0]);

    // Build the matrix to convert contact impulse to change in velocity
    // in world coordinates.
    glm::dmat3 deltaVelWorld = glm::dmat3(1.0);
    deltaVelWorld = impulseToTorque * inverseInertiaTensor[0] * impulseToTorque * -1;

    // Check if we need to add body two's mat
    if (body[1])
    {
        // Set the cross product matrix
        impulseToTorque = setSkewSymmetric(impulseToTorque, relativeContactPosition[1]);

        // Calculate the velocity change matrix
        glm::dmat3 deltaVelWorld2 = glm::dmat3(1.0);
        deltaVelWorld2 = impulseToTorque * inverseInertiaTensor[1] * impulseToTorque * -1;

        // Add to the total delta velocity.
        deltaVelWorld += deltaVelWorld2;

        // Add to the inverse mass
        inverseMass += body[1]->inverseMass;
    }

    // Do a change of basis to convert into contact coordinates.
    glm::dmat3 deltaVelocity = glm::dmat3(1.0);
    deltaVelocity = glm::transpose(contactToWorld) * deltaVelWorld * contactToWorld;
    // Add in the linear velocity change
    deltaVelocity[0][0] += inverseMass;
    deltaVelocity[1][1] += inverseMass;
    deltaVelocity[2][2] += inverseMass;

    // Invert to get the impulse needed per unit velocity
    glm::dmat3 impulseMatrix = glm::inverse(deltaVelocity);

    // Find the target velocities to kill
    glm::dvec3 velKill(desiredDeltaVelocity,
        -contactVelocity.y,
        -contactVelocity.z);

    // Find the impulse to kill target velocities
    impulseContact = velKill * impulseMatrix;

    // Check for exceeding friction
    double planarImpulse = sqrt(
        impulseContact.y*impulseContact.y +
        impulseContact.z*impulseContact.z
        );
    if (planarImpulse > impulseContact.x * friction)
    {
        // We need to use dynamic friction
        impulseContact.y /= planarImpulse;
        impulseContact.z /= planarImpulse;

        impulseContact.x = deltaVelocity[0][0] +
            deltaVelocity[0][1]*friction*impulseContact.y +
            deltaVelocity[0][2]*friction*impulseContact.z;
        impulseContact.x = desiredDeltaVelocity / impulseContact.x;
        impulseContact.y *= friction * impulseContact.x;
        impulseContact.z *= friction * impulseContact.x;
    }
    return impulseContact;
}

void Contact::applyPositionChange(glm::dvec3 linearChange[2],
                                  glm::dvec3 angularChange[2],
                                  double penetration)
{
    const double angularLimit = (double)0.2f;
    double angularMove[2];
    double linearMove[2];

    double totalInertia = 0;
    double linearInertia[2];
    double angularInertia[2];

    // We need to work out the inertia of each object in the direction
    // of the contact normal, due to angular inertia only.
    for (unsigned i = 0; i < 2; i++) if (body[i])
    {
        glm::dmat3 inverseInertiaTensor = body[i]->inverseInertiaTensorWorld;

        // Use the same procedure as for calculating frictionless
        // velocity change to work out the angular inertia.
        glm::dvec3 angularInertiaWorld =
            cross(relativeContactPosition[i], contactNormal);
        angularInertiaWorld =
            angularInertiaWorld * inverseInertiaTensor;
        angularInertiaWorld =
            cross(angularInertiaWorld, relativeContactPosition[i]);
        angularInertia[i] =
            dot(angularInertiaWorld, contactNormal);

        // The linear component is simply the inverse mass
        linearInertia[i] = body[i]->inverseMass;

        // Keep track of the total inertia from all components
        totalInertia += linearInertia[i] + angularInertia[i];

        // We break the loop here so that the totalInertia value is
        // completely calculated (by both iterations) before
        // continuing.
    }

    // Loop through again calculating and applying the changes
    for (unsigned i = 0; i < 2; i++) if (body[i])
    {
        // The linear and angular movements required are in proportion to
        // the two inverse inertias.
        double sign = (i == 0)?1:-1;
        angularMove[i] =
            sign * penetration * (angularInertia[i] / totalInertia);
        linearMove[i] =
            sign * penetration * (linearInertia[i] / totalInertia);

        // To avoid angular projections that are too great (when mass is large
        // but inertia tensor is small) limit the angular move.
        glm::dvec3 projection = relativeContactPosition[i];
        projection += 
            contactNormal *
            -dot(relativeContactPosition[i], contactNormal);

        // Use the small angle approximation for the sine of the angle (i.e.
        // the magnitude would be sine(angularLimit) * projection.magnitude
        // but we approximate sine(angularLimit) to angularLimit).
        double maxMagnitude = angularLimit * length(projection);

        if (angularMove[i] < -maxMagnitude)
        {
            double totalMove = angularMove[i] + linearMove[i];
            angularMove[i] = -maxMagnitude;
            linearMove[i] = totalMove - angularMove[i];
        }
        else if (angularMove[i] > maxMagnitude)
        {
            double totalMove = angularMove[i] + linearMove[i];
            angularMove[i] = maxMagnitude;
            linearMove[i] = totalMove - angularMove[i];
        }

        // We have the linear amount of movement required by turning
        // the rigid body (in angularMove[i]). We now need to
        // calculate the desired rotation to achieve that.
        if (angularMove[i] == 0)
        {
            // Easy case - no angular movement means no rotation.
            angularChange[i] = glm::dvec3(0.0, 0.0, 0.0);
        }
        else
        {
            // Work out the direction we'd like to rotate in.
            glm::dvec3 targetAngularDirection =
                cross(relativeContactPosition[i], contactNormal);

            glm::dmat3 inverseInertiaTensor = body[i]->inverseInertiaTensorWorld;

            // Work out the direction we'd need to rotate to achieve that
            angularChange[i] =
                targetAngularDirection * inverseInertiaTensor *
                (angularMove[i] / angularInertia[i]);
        }

        // Velocity change is easier - it is just the linear movement
        // along the contact normal.
        linearChange[i] = contactNormal * linearMove[i];

        // Now we can start to apply the values we've calculated.
        // Apply the linear movement
        glm::dvec3 pos = body[i]->position;
        pos += contactNormal * linearMove[i];
        body[i]->position = pos;

        // And the change in orientation
        glm::dquat q = body[i]->orientation;
        q += glm::dquat(0, angularChange[i]) * q * 0.5;
        body[i]->orientation = q;

        // We need to calculate the derived mat for any body that is
        // asleep, so that the changes are reflected in the object's
        // mat. Otherwise the resolution will not change the position
        // of the object, and the next collision detection round will
        // have the same penetration.
        if (!body[i]->isAwake) {
            body[i]->orientation = normalize(body[i]->orientation);

            // Calculate the transform matrix for the body.
            body[i]->transformMatrix = (glm::dmat4)body[i]->orientation;
            body[i]->transformMatrix[3] = glm::dvec4(body[i]->position, 1.0);
            // Calculate the inertiaTensor in world space.
            body[i]->inverseInertiaTensorWorld = glm::dmat3(body[i]->transformMatrix) * body[i]->inverseInertiaTensor * glm::dmat3(glm::inverse(body[i]->transformMatrix));

        };
    }
}


// Contact resolver implementation

ContactResolver::ContactResolver(unsigned iterations,
                                 double velocityEpsilon,
                                 double positionEpsilon)
{
    setIterations(iterations, iterations);
    setEpsilon(velocityEpsilon, positionEpsilon);
}

void ContactResolver::setIterations(unsigned velocityIterations,
                                    unsigned positionIterations)
{
    ContactResolver::velocityIterations = velocityIterations;
    ContactResolver::positionIterations = positionIterations;
}

void ContactResolver::setEpsilon(double velocityEpsilon,
                                 double positionEpsilon)
{
    ContactResolver::velocityEpsilon = velocityEpsilon;
    ContactResolver::positionEpsilon = positionEpsilon;
}

void ContactResolver::resolveContacts(Contact *contacts,
                                      unsigned numContacts,
                                      double duration)
{
    // Make sure we have something to do.
    if (numContacts == 0) return;
    if (!isValid()) return;

    // Prepare the contacts for processing
    prepareContacts(contacts, numContacts, duration);

    // Resolve the interpenetration problems with the contacts.
    adjustPositions(contacts, numContacts, duration);

    // Resolve the velocity problems with the contacts.
    adjustVelocities(contacts, numContacts, duration);
}

void ContactResolver::prepareContacts(Contact* contacts,
                                      unsigned numContacts,
                                      double duration)
{
    // Generate contact velocity and axis information.
    Contact* lastContact = contacts + numContacts;
    for (Contact* contact=contacts; contact < lastContact; contact++)
    {
        // Calculate the internal contact mat (inertia, basis, etc).
        contact->calculateInternals(duration);
    }
}

void ContactResolver::adjustVelocities(Contact *c,
                                       unsigned numContacts,
                                       double duration)
{
    glm::dvec3 velocityChange[2], rotationChange[2];
    glm::dvec3 deltaVel;

    // iteratively handle impacts in order of severity.
    velocityIterationsUsed = 0;
    while (velocityIterationsUsed < velocityIterations)
    {
        // Find contact with maximum magnitude of probable velocity change.
        double max = velocityEpsilon;
        unsigned index = numContacts;
        for (unsigned i = 0; i < numContacts; i++)
        {
            if (c[i].desiredDeltaVelocity > max)
            {
                max = c[i].desiredDeltaVelocity;
                index = i;
            }
        }
        if (index == numContacts) break;

        // Match the awake state at the contact
        c[index].matchAwakeState();

        // Do the resolution on the contact that came out top.
        c[index].applyVelocityChange(velocityChange, rotationChange);

        // With the change in velocity of the two bodies, the update of
        // contact velocities means that some of the relative closing
        // velocities need recomputing.
        for (unsigned i = 0; i < numContacts; i++)
        {
            // Check each body in the contact
            for (unsigned b = 0; b < 2; b++) if (c[i].body[b])
            {
                // Check for a match with each body in the newly
                // resolved contact
                for (unsigned d = 0; d < 2; d++)
                {
                    if (c[i].body[b] == c[index].body[d])
                    {
                        deltaVel = velocityChange[d] +
                            cross(rotationChange[d], 
                                c[i].relativeContactPosition[b]);

                        // The sign of the change is negative if we're dealing
                        // with the second body in a contact.
                        c[i].contactVelocity +=
                            glm::transpose(c[i].contactToWorld) * deltaVel
                            * (b?-1.0:1.0);
                        c[i].calculateDesiredDeltaVelocity(duration);
                    }
                }
            }
        }
        velocityIterationsUsed++;
    }
}

void ContactResolver::adjustPositions(Contact *c,
                                      unsigned numContacts,
                                      double duration)
{
    unsigned i,index;
    glm::dvec3 linearChange[2], angularChange[2];
    double max;
    glm::dvec3 deltaPosition;

    // iteratively resolve interpenetrations in order of severity.
    positionIterationsUsed = 0;
    while (positionIterationsUsed < positionIterations)
    {
        // Find biggest penetration
        max = positionEpsilon;
        index = numContacts;
        for (i=0; i<numContacts; i++)
        {
            if (c[i].penetration > max)
            {
                max = c[i].penetration;
                index = i;
            }
        }
        if (index == numContacts) break;

        // Match the awake state at the contact
        c[index].matchAwakeState();

        // Resolve the penetration.
        c[index].applyPositionChange(
            linearChange,
            angularChange,
            max);

        // Again this action may have changed the penetration of other
        // bodies, so we update contacts.
        for (i = 0; i < numContacts; i++)
        {
            // Check each body in the contact
            for (unsigned b = 0; b < 2; b++) if (c[i].body[b])
            {
                // Check for a match with each body in the newly
                // resolved contact
                for (unsigned d = 0; d < 2; d++)
                {
                    if (c[i].body[b] == c[index].body[d])
                    {
                        deltaPosition = linearChange[d] +
                            cross(angularChange[d],
                                c[i].relativeContactPosition[b]);

                        // The sign of the change is positive if we're
                        // dealing with the second body in a contact
                        // and negative otherwise (because we're
                        // subtracting the resolution)..
                        c[i].penetration +=
                            dot(deltaPosition, c[i].contactNormal)
                            * (b?1:-1);
                    }
                }
            }
        }
        positionIterationsUsed++;
    }
}
