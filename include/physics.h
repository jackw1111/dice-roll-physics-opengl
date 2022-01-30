
#ifndef PHYSICS_H
#define PHYSICS_H 

#include <math.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <vector>
#include <float.h>
#include <memory.h>
#include <assert.h>

#define DOUBLE_PRECISION 

extern double sleepEpsilon;

glm::dvec3 getAxisVector(glm::dmat4 m, int i);


glm::dmat3 setSkewSymmetric(glm::dmat3 m1, const glm::dvec3 vector);

glm::dmat3 setBlockInertiaTensor(glm::dmat3 m, const glm::dvec3 &halfSizes, double mass);

glm::dmat3 setInertiaTensorCoeffs(glm::dmat3 m, double ix, double iy, double iz,
    double ixy=0, double ixz=0, double iyz=0);

class RigidBody
{
public:
    double inverseMass;

    glm::dmat3 inverseInertiaTensor = glm::dmat3(1.0);
    glm::dmat3 inverseInertiaTensorWorld = glm::dmat3(1.0);

    double linearDamping;
    double angularDamping;

    glm::dvec3 position;
    glm::dvec3 velocity;
    glm::dquat orientation;
    glm::dvec3 rotation;
    glm::dvec3 acceleration;
    glm::dvec3 lastFrameAcceleration;
    glm::dmat4 transformMatrix = glm::dmat4(1.0);

    double motion;
    bool isAwake;
    bool canSleep;

    glm::dvec3 forceAccum;
    glm::dvec3 torqueAccum;

public:
    void integrate(double duration);
    void setMass(const double mass);
    void setAwake(const bool awake=true);
    void setCanSleep(const bool canSleep=true);

};

class ContactResolver;
class Contact
{

    friend class ContactResolver;

public:

    RigidBody* body[2];
    double friction;
    double restitution;
    glm::dvec3 contactPoint;
    glm::dvec3 contactNormal;
    double penetration;

    void setBodyData(RigidBody* one, RigidBody *two,
                     double friction, double restitution);

protected:

    glm::dmat3 contactToWorld = glm::dmat3(1.0);

    glm::dvec3 contactVelocity;

    double desiredDeltaVelocity;

    glm::dvec3 relativeContactPosition[2];

protected:

    void calculateInternals(double duration);

    void swapBodies();

    void matchAwakeState();

    void calculateDesiredDeltaVelocity(double duration);

    glm::dvec3 calculateLocalVelocity(unsigned bodyIndex, double duration);

    void calculateContactBasis();

    void applyImpulse(const glm::dvec3 &impulse, RigidBody *body,
                      glm::dvec3 *velocityChange, glm::dvec3 *rotationChange);

    void applyVelocityChange(glm::dvec3 velocityChange[2],
                             glm::dvec3 rotationChange[2]);

    void applyPositionChange(glm::dvec3 linearChange[2],
                             glm::dvec3 angularChange[2],
                             double penetration);
    glm::dvec3 calculateFrictionlessImpulse(glm::dmat3 *inverseInertiaTensor);
    glm::dvec3 calculateFrictionImpulse(glm::dmat3 *inverseInertiaTensor);
};
class ContactResolver
{
protected:

    unsigned velocityIterations;

    unsigned positionIterations;
    double velocityEpsilon;
    double positionEpsilon;

public:

    unsigned velocityIterationsUsed;

    unsigned positionIterationsUsed;

private:
    bool validSettings;

public:
    ContactResolver(){};
    
    ContactResolver(unsigned iterations,
        double velocityEpsilon=(double)0.01,
        double positionEpsilon=(double)0.01);

    ContactResolver(unsigned velocityIterations,
        unsigned positionIterations,
        double velocityEpsilon=(double)0.01,
        double positionEpsilon=(double)0.01);

    bool isValid()
    {
        return (velocityIterations > 0) &&
               (positionIterations > 0) &&
               (positionEpsilon >= 0.0f) &&
               (positionEpsilon >= 0.0f);
    }

    void setIterations(unsigned velocityIterations,
                       unsigned positionIterations);

    void setIterations(unsigned iterations);

    void setEpsilon(double velocityEpsilon,
                    double positionEpsilon);
    void resolveContacts(Contact *contactArray,
        unsigned numContacts,
        double duration);

protected:

    void prepareContacts(Contact *contactArray, unsigned numContacts,
        double duration);

    void adjustVelocities(Contact *contactArray,
        unsigned numContacts,
        double duration);

    void adjustPositions(Contact *contacts,
        unsigned numContacts,
        double duration);
};

class ContactGenerator
{
public:
    virtual unsigned addContact(Contact *contact, unsigned limit) const = 0;
};

class IntersectionTests;
class CollisionDetector;


class CollisionBox
{
public:
    static CollisionBox* boxData[256];
    static unsigned int boxCount;

    CollisionBox(double x, double y, double z, glm::vec3 orientation, double gravity, double rx, double ry, double rz, double sx, double sy, double sz);

    friend class IntersectionTests;
    friend class CollisionDetector;

    RigidBody *body;

    glm::dmat4 offset = glm::dmat4(1.0);

    unsigned int boxID;

    void calculateInternals();

    glm::dvec3 getAxis(unsigned index) const
    {
        return getAxisVector(transform, index);
    }

    const glm::dmat4& getTransform() const
    {
        return transform;
    }

    glm::dmat4 transform = glm::dmat4(1.0);

    glm::mat4 modelMatrix = glm::mat4(1.0f);

    glm::dvec3 halfSize;

    /** Sets the box to a specific location. */
    void setState(double x, double y, double z, glm::vec3 orientation, double gravity, double rx, double ry, double rz, double sx, double sy, double sz);

    glm::mat4 getModelMatrix();
    void setModelMatrix(glm::mat4 m);
};

class IntersectionTests
{
public:

    static bool boxAndBox(
        const CollisionBox &one,
        const CollisionBox &two);
};

struct CollisionData
{

    Contact *contactArray;

    Contact *contacts;

    int contactsLeft;

    unsigned contactCount;

    double friction;

    double restitution;

    double tolerance;

    bool hasMoreContacts()
    {
        return contactsLeft > 0;
    }

    void reset(unsigned maxContacts)
    {
        contactsLeft = maxContacts;
        contactCount = 0;
        contacts = contactArray;
    }

    void addContacts(unsigned count)
    {

        contactsLeft -= count;
        contactCount += count;
        contacts += count;
    }
};

class CollisionDetector
{
public:

    static unsigned boxAndBox(
        const CollisionBox &one,
        const CollisionBox &two,
        CollisionData *data
        );
};

#endif
