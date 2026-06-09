#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystem.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/EstimateCollisionResponse.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/CompoundShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/PlaneShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/TriangleShape.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/SimShapeFilter.h>
#include <Jolt/Physics/Constraints/ConstraintPart/SwingTwistConstraintPart.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/DistanceConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include <Jolt/Renderer/DebugRendererSimple.h>

#include <Jolt/Physics/Character/Character.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>

#include <Jolt/Geometry/AABox.h>
#include <Jolt/Physics/Collision/PhysicsMaterial.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCylinderShape.h>
#include <Jolt/Physics/Collision/Shape/EmptyShape.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/Constraints/ConeConstraint.h>
#include <Jolt/Physics/Constraints/PulleyConstraint.h>
#include <Jolt/Physics/Constraints/GearConstraint.h>
#include <Jolt/Physics/Constraints/RackAndPinionConstraint.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/Constraints/PathConstraint.h>
#include <Jolt/Physics/Constraints/PathConstraintPathHermite.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>
#include <Jolt/Physics/Vehicle/VehicleCollisionTester.h>
#include <Jolt/Physics/Vehicle/VehicleEngine.h>
#include <Jolt/Physics/Vehicle/VehicleTransmission.h>
#include <Jolt/Physics/Vehicle/VehicleDifferential.h>
#include <Jolt/Physics/Vehicle/VehicleAntiRollBar.h>
#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyContactListener.h>
#include <Jolt/Physics/SoftBody/SoftBodyManifold.h>
#include <Jolt/Physics/SoftBody/SoftBodyVertex.h>
#include <Jolt/Physics/StateRecorder.h>
#include <Jolt/Physics/StateRecorderImpl.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>

#include <JoltC/JoltC.h>

#define JPC_IMPL static

#define OPAQUE_WRAPPER(c_type, cpp_type) \
	static c_type* to_jpc(cpp_type *in) { return reinterpret_cast<c_type*>(in); } \
	static const c_type* to_jpc(const cpp_type *in) { return reinterpret_cast<const c_type*>(in); } \
	static cpp_type* to_jph(c_type *in) { return reinterpret_cast<cpp_type*>(in); } \
	static const cpp_type* to_jph(const c_type *in) { return reinterpret_cast<const cpp_type*>(in); } \
	static cpp_type** to_jph(c_type **in) { return reinterpret_cast<cpp_type**>(in); }

#define DESTRUCTOR(c_type) \
	JPC_API void c_type##_delete(c_type* object) { \
		delete to_jph(object); \
	}

#define ENUM_CONVERSION(c_type, cpp_type) \
	static c_type to_jpc(cpp_type in) { return static_cast<c_type>(in); } \
	static cpp_type to_jph(c_type in) { return static_cast<cpp_type>(in); }

#define LAYOUT_COMPATIBLE(c_type, cpp_type) \
	static c_type to_jpc(cpp_type in) { \
		c_type out; \
		memcpy(&out, &in, sizeof(c_type)); \
		return out; \
	} \
	static cpp_type to_jph(c_type in) { \
		cpp_type out; \
		memcpy(&out, &in, sizeof(cpp_type)); \
		return out; \
	} \
	static c_type* to_jpc(cpp_type* in) { \
		return reinterpret_cast<c_type*>(in); \
	} \
	static cpp_type* to_jph(c_type* in) { \
		return reinterpret_cast<cpp_type*>(in); \
	} \
	static const c_type* to_jpc(const cpp_type* in) { \
		return reinterpret_cast<const c_type*>(in); \
	} \
	static const cpp_type* to_jph(const c_type* in) { \
		return reinterpret_cast<const cpp_type*>(in); \
	} \
	static_assert(sizeof(c_type) == sizeof(cpp_type), "size of " #c_type " did not match size of " #cpp_type); \
	static_assert(alignof(c_type) == alignof(cpp_type), "align of " #c_type " did not match align of " #cpp_type); \
	static_assert(!std::is_polymorphic_v<cpp_type>, #cpp_type " is polymorphic and cannot be made layout compatible");

template<typename E>
constexpr auto to_integral(E e) -> typename std::underlying_type<E>::type
{
	return static_cast<typename std::underlying_type<E>::type>(e);
}

ENUM_CONVERSION(JPC_MotionType, JPH::EMotionType)
ENUM_CONVERSION(JPC_AllowedDOFs, JPH::EAllowedDOFs)
ENUM_CONVERSION(JPC_Activation, JPH::EActivation)
ENUM_CONVERSION(JPC_BodyType, JPH::EBodyType)
ENUM_CONVERSION(JPC_MotionQuality, JPH::EMotionQuality)
ENUM_CONVERSION(JPC_OverrideMassProperties, JPH::EOverrideMassProperties)
ENUM_CONVERSION(JPC_ShapeType, JPH::EShapeType)
ENUM_CONVERSION(JPC_ShapeSubType, JPH::EShapeSubType)
ENUM_CONVERSION(JPC_SpringMode, JPH::ESpringMode)
ENUM_CONVERSION(JPC_MotorState, JPH::EMotorState)
ENUM_CONVERSION(JPC_ValidateResult, JPH::ValidateResult)

OPAQUE_WRAPPER(JPC_PhysicsSystem, JPH::PhysicsSystem)
DESTRUCTOR(JPC_PhysicsSystem)

OPAQUE_WRAPPER(JPC_BodyInterface, JPH::BodyInterface)
OPAQUE_WRAPPER(JPC_BodyLockInterface, JPH::BodyLockInterface)
OPAQUE_WRAPPER(JPC_BodyLockRead, JPH::BodyLockRead)
OPAQUE_WRAPPER(JPC_BodyLockWrite, JPH::BodyLockWrite)
OPAQUE_WRAPPER(JPC_BodyLockMultiRead, JPH::BodyLockMultiRead)
OPAQUE_WRAPPER(JPC_BodyLockMultiWrite, JPH::BodyLockMultiWrite)
OPAQUE_WRAPPER(JPC_NarrowPhaseQuery, JPH::NarrowPhaseQuery)

OPAQUE_WRAPPER(JPC_TempAllocatorImpl, JPH::TempAllocatorImpl)
DESTRUCTOR(JPC_TempAllocatorImpl)

OPAQUE_WRAPPER(JPC_JobSystem, JPH::JobSystem)
DESTRUCTOR(JPC_JobSystem)

OPAQUE_WRAPPER(JPC_JobSystemThreadPool, JPH::JobSystemThreadPool)
DESTRUCTOR(JPC_JobSystemThreadPool)

OPAQUE_WRAPPER(JPC_JobSystemSingleThreaded, JPH::JobSystemSingleThreaded)
DESTRUCTOR(JPC_JobSystemSingleThreaded)

OPAQUE_WRAPPER(JPC_Shape, JPH::Shape)
OPAQUE_WRAPPER(JPC_CompoundShape, JPH::CompoundShape)
OPAQUE_WRAPPER(JPC_Body, JPH::Body)

OPAQUE_WRAPPER(JPC_VertexList, JPH::VertexList)
DESTRUCTOR(JPC_VertexList)

OPAQUE_WRAPPER(JPC_IndexedTriangleList, JPH::IndexedTriangleList)
DESTRUCTOR(JPC_IndexedTriangleList)

OPAQUE_WRAPPER(JPC_String, JPH::String)
DESTRUCTOR(JPC_String)

LAYOUT_COMPATIBLE(JPC_BodyManager_DrawSettings, JPH::BodyManager::DrawSettings)

LAYOUT_COMPATIBLE(JPC_ShapeCastSettings, JPH::ShapeCastSettings)
LAYOUT_COMPATIBLE(JPC_CollideShapeSettings, JPH::CollideShapeSettings)

LAYOUT_COMPATIBLE(JPC_BodyID, JPH::BodyID)

static auto to_jpc(JPH::BroadPhaseLayer in) { return in.GetValue(); }
static auto to_jph(JPC_BroadPhaseLayer in) { return JPH::BroadPhaseLayer(in); }

static JPC_Vec2 to_jpc(JPH::Vector<2> in) {
	return JPC_Vec2{in[0], in[1]};
}
static JPH::Vector<2> to_jph(JPC_Vec2 in) {
	JPH::Vector<2> out;
	out[0] = in.x;
	out[1] = in.y;
	return out;
}

static JPC_Vec3 to_jpc(JPH::Vec3 in) {
	return JPC_Vec3{in.GetX(), in.GetY(), in.GetZ(), in.GetZ()};
}
static JPH::Vec3 to_jph(JPC_Vec3 in) {
	return JPH::Vec3(in.x, in.y, in.z);
}

static JPC_Vec4 to_jpc(JPH::Vec4 in) {
	return JPC_Vec4{in.GetX(), in.GetY(), in.GetZ(), in.GetW()};
}
static JPH::Vec4 to_jph(JPC_Vec4 in) {
	return JPH::Vec4(in.x, in.y, in.z, in.w);
}

static JPH::Array<JPH::Vec3> to_jph(const JPC_Vec3* src, size_t n) {
	JPH::Array<JPH::Vec3> vec;
	vec.resize(n);

	if (src != nullptr) {
		memcpy(vec.data(), src, n * sizeof(*src));
	}

	return vec;
}

static JPC_DVec3 to_jpc(JPH::DVec3 in) {
	return JPC_DVec3{in.GetX(), in.GetY(), in.GetZ(), in.GetZ()};
}
static JPH::DVec3 to_jph(JPC_DVec3 in) {
	return JPH::DVec3(in.x, in.y, in.z);
}

static JPC_Quat to_jpc(JPH::Quat in) {
	return JPC_Quat{in.GetX(), in.GetY(), in.GetZ(), in.GetW()};
}
static JPH::Quat to_jph(JPC_Quat in) {
	return JPH::Quat(in.x, in.y, in.z, in.w);
}

static JPC_Mat44 to_jpc(JPH::Mat44 in) {
	JPC_Mat44 out;
	in.StoreFloat4x4(reinterpret_cast<JPH::Float4*>(&out));
	return out;
}
static JPH::Mat44 to_jph(JPC_Mat44 in) {
	return JPH::Mat44::sLoadFloat4x4Aligned(reinterpret_cast<const JPH::Float4*>(&in));
}

static JPC_DMat44 to_jpc(JPH::DMat44 in) {
	JPC_DMat44 out;
	out.col[0] = to_jpc(in.GetColumn4(0));
	out.col[1] = to_jpc(in.GetColumn4(1));
	out.col[2] = to_jpc(in.GetColumn4(2));
	out.col3 = to_jpc(in.GetTranslation());
	return out;
}
static JPH::DMat44 to_jph(JPC_DMat44 in) {
	JPH::DVec3 col3 = to_jph(in.col3);

	JPH::DMat44 out(
		to_jph(in.col[0]),
		to_jph(in.col[1]),
		to_jph(in.col[2]),
		col3);
	return out;
}

static JPC_Color to_jpc(JPH::Color in) {
	return JPC_Color{in.r, in.g, in.b, in.a};
}
static JPH::Color to_jph(JPC_Color in) {
	return JPH::Color(in.r, in.g, in.b, in.a);
}

static JPH::RayCast to_jph(JPC_RayCast in) {
	return JPH::RayCast(to_jph(in.Origin), to_jph(in.Direction));
}

static JPH::RRayCast to_jph(JPC_RRayCast in) {
	return JPH::RRayCast(to_jph(in.Origin), to_jph(in.Direction));
}

static JPH::RShapeCast to_jph(JPC_RShapeCast in) {
	return JPH::RShapeCast(
		to_jph(in.Shape),
		to_jph(in.Scale),
		to_jph(in.CenterOfMassStart),
		to_jph(in.Direction));
}

static JPH::SubShapeID JPC_SubShapeID_to_jph(JPC_SubShapeID in) {
	JPH::SubShapeID out;
	out.SetValue(in);
	return out;
}

static JPC_SubShapeID to_jpc(JPH::SubShapeID in) {
	return in.GetValue();
}

static JPC_RayCastResult to_jpc(JPH::RayCastResult in) {
	JPC_RayCastResult out{0};
	out.BodyID = to_jpc(in.mBodyID);
	out.Fraction = in.mFraction;
	out.SubShapeID2 = to_jpc(in.mSubShapeID2);

	return out;
}

JPC_IMPL JPC_ShapeCastResult JPC_ShapeCastResult_to_jpc(JPH::ShapeCastResult in) {
	JPC_ShapeCastResult out{};
	// CollideShapeResult
	out.ContactPointOn1 = to_jpc(in.mContactPointOn1);
	out.ContactPointOn2 = to_jpc(in.mContactPointOn2);
	out.PenetrationAxis = to_jpc(in.mPenetrationAxis);
	out.PenetrationDepth = in.mPenetrationDepth;
	out.SubShapeID1 = to_jpc(in.mSubShapeID1);
	out.SubShapeID2 = to_jpc(in.mSubShapeID2);
	out.BodyID2 = to_jpc(in.mBodyID2);
	// Face Shape1Face;
	// Face Shape2Face;

	// ShapeCastResult
	out.Fraction = in.mFraction;
	out.IsBackFaceHit = in.mIsBackFaceHit;

	return out;
}

JPC_IMPL JPH::ShapeCastSettings JPC_ShapeCastSettings_to_jph(JPC_ShapeCastSettings in) {
	JPH::ShapeCastSettings out{};

	// JPH::CollideSettingsBase
	// EActiveEdgeMode ActiveEdgeMode;
	// ECollectFacesMode CollectFacesMode;
	out.mCollisionTolerance = in.CollisionTolerance;
	out.mPenetrationTolerance = in.PenetrationTolerance;
	out.mActiveEdgeMovementDirection = to_jph(in.ActiveEdgeMovementDirection);

	// JPH::ShapeCastSettings
	out.mBackFaceModeTriangles = static_cast<JPH::EBackFaceMode>(in.BackFaceModeTriangles);
	out.mBackFaceModeConvex = static_cast<JPH::EBackFaceMode>(in.BackFaceModeConvex);
	out.mUseShrunkenShapeAndConvexRadius = in.UseShrunkenShapeAndConvexRadius;
	out.mReturnDeepestPoint = in.ReturnDeepestPoint;

	return out;
}

JPC_IMPL JPC_CollideShapeResult JPC_CollideShapeResult_to_jpc(JPH::CollideShapeResult in) {
	JPC_CollideShapeResult out{};
	// CollideShapeResult
	out.ContactPointOn1 = to_jpc(in.mContactPointOn1);
	out.ContactPointOn2 = to_jpc(in.mContactPointOn2);
	out.PenetrationAxis = to_jpc(in.mPenetrationAxis);
	out.PenetrationDepth = in.mPenetrationDepth;
	out.SubShapeID1 = to_jpc(in.mSubShapeID1);
	out.SubShapeID2 = to_jpc(in.mSubShapeID2);
	out.BodyID2 = to_jpc(in.mBodyID2);
	// Face Shape1Face;
	// Face Shape2Face;

	return out;
}

JPC_API void JPC_RegisterDefaultAllocator() {
	JPH::RegisterDefaultAllocator();
}

JPC_API void JPC_FactoryInit() {
	JPH::Factory::sInstance = new JPH::Factory();
}

JPC_API void JPC_FactoryDelete() {
	delete JPH::Factory::sInstance;
	JPH::Factory::sInstance = nullptr;
}

JPC_API void JPC_RegisterTypes() {
	JPH::RegisterTypes();
}

JPC_API void JPC_UnregisterTypes() {
	JPH::UnregisterTypes();
}

////////////////////////////////////////////////////////////////////////////////
// VertexList == Array<Float3> == std::vector<Float3>

JPC_API JPC_VertexList* JPC_VertexList_new(const JPC_Float3* storage, size_t len) {
	const JPH::Float3* new_storage = (const JPH::Float3*)storage;
	return to_jpc(new JPH::VertexList(new_storage, new_storage + len));
}

////////////////////////////////////////////////////////////////////////////////
// IndexedTriangleList == Array<IndexedTriangle> == std::vector<IndexedTriangle>

JPC_API JPC_IndexedTriangleList* JPC_IndexedTriangleList_new(const JPC_IndexedTriangle* storage, size_t len) {
	const JPH::IndexedTriangle* new_storage = (const JPH::IndexedTriangle*)storage;
	return to_jpc(new JPH::IndexedTriangleList(new_storage, new_storage + len));
}

////////////////////////////////////////////////////////////////////////////////
// TempAllocatorImpl

JPC_API JPC_TempAllocatorImpl* JPC_TempAllocatorImpl_new(uint size) {
	return to_jpc(new JPH::TempAllocatorImpl(size));
}

////////////////////////////////////////////////////////////////////////////////
// JobSystemThreadPool

JPC_API JPC_JobSystemThreadPool* JPC_JobSystemThreadPool_new2(
	uint inMaxJobs,
	uint inMaxBarriers)
{
	return to_jpc(new JPH::JobSystemThreadPool(inMaxJobs, inMaxBarriers));
}

JPC_API JPC_JobSystemThreadPool* JPC_JobSystemThreadPool_new3(
	uint inMaxJobs,
	uint inMaxBarriers,
	int inNumThreads)
{
	return to_jpc(new JPH::JobSystemThreadPool(inMaxJobs, inMaxBarriers, inNumThreads));
}

////////////////////////////////////////////////////////////////////////////////
// JobSystemSingleThreaded

JPC_API JPC_JobSystemSingleThreaded* JPC_JobSystemSingleThreaded_new(uint inMaxJobs) {
	return to_jpc(new JPH::JobSystemSingleThreaded(inMaxJobs));
}

////////////////////////////////////////////////////////////////////////////////
// CollisionGroup

JPC_IMPL JPC_CollisionGroup JPC_CollisionGroup_to_jpc(const JPH::CollisionGroup* input);

class JPC_GroupFilterBridge final : public JPH::GroupFilter {
public:
	explicit JPC_GroupFilterBridge(const void *self, JPC_GroupFilterFns fns) : self(self), fns(fns) {}

	bool CanCollide(const JPH::CollisionGroup &inGroup1, const JPH::CollisionGroup &inGroup2) const override {
		JPC_CollisionGroup jpcGroup1 = JPC_CollisionGroup_to_jpc(&inGroup1);
		JPC_CollisionGroup jpcGroup2 = JPC_CollisionGroup_to_jpc(&inGroup2);

		return fns.CanCollide(self, &jpcGroup1, &jpcGroup2);
	}

	void SaveBinaryState([[maybe_unused]] JPH::StreamOut &inStream) const override {}
	void RestoreBinaryState([[maybe_unused]] JPH::StreamIn &inStream) override {}

private:
	const void* self;
	JPC_GroupFilterFns fns;
};

OPAQUE_WRAPPER(JPC_GroupFilter, JPC_GroupFilterBridge)
DESTRUCTOR(JPC_GroupFilter)

JPC_IMPL JPH::CollisionGroup JPC_CollisionGroup_to_jph(const JPC_CollisionGroup* self) {
	const JPC_GroupFilterBridge* filter_group = to_jph(self->GroupFilter);

	JPH::CollisionGroup group(filter_group, self->GroupID, self->SubGroupID);
	return group;
}

JPC_IMPL JPC_CollisionGroup JPC_CollisionGroup_to_jpc(const JPH::CollisionGroup* input) {
	JPC_CollisionGroup group{};
	group.GroupFilter; // NOTE: This member doesn't matter for callers of this function
	group.GroupID = input->GetGroupID();
	group.SubGroupID = input->GetSubGroupID();
	return group;
}

JPC_API JPC_GroupFilter* JPC_GroupFilter_new(
	const void *self,
	JPC_GroupFilterFns fns)
{
	return to_jpc(new JPC_GroupFilterBridge(self, fns));
}

////////////////////////////////////////////////////////////////////////////////
// BroadPhaseLayerInterface

class JPC_BroadPhaseLayerInterfaceBridge final : public JPH::BroadPhaseLayerInterface {
public:
	explicit JPC_BroadPhaseLayerInterfaceBridge(const void *self, JPC_BroadPhaseLayerInterfaceFns fns) : self(self), fns(fns) {}

	virtual uint GetNumBroadPhaseLayers() const override {
		return fns.GetNumBroadPhaseLayers(self);
	}

	virtual JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override {
		return to_jph(fns.GetBroadPhaseLayer(self, inLayer));
	}

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
	virtual const char * GetBroadPhaseLayerName([[maybe_unused]] JPH::BroadPhaseLayer inLayer) const override {
		return "FIXME";
	}
#endif

private:
	const void* self;
	JPC_BroadPhaseLayerInterfaceFns fns;
};

OPAQUE_WRAPPER(JPC_BroadPhaseLayerInterface, JPC_BroadPhaseLayerInterfaceBridge)
DESTRUCTOR(JPC_BroadPhaseLayerInterface)

JPC_API JPC_BroadPhaseLayerInterface* JPC_BroadPhaseLayerInterface_new(
	const void *self,
	JPC_BroadPhaseLayerInterfaceFns fns)
{
	return to_jpc(new JPC_BroadPhaseLayerInterfaceBridge(self, fns));
}

////////////////////////////////////////////////////////////////////////////////
// ObjectVsBroadPhaseLayerFilter

class JPC_ObjectVsBroadPhaseLayerFilterBridge final : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
	explicit JPC_ObjectVsBroadPhaseLayerFilterBridge(const void *self, JPC_ObjectVsBroadPhaseLayerFilterFns fns) : self(self), fns(fns) {}

	virtual bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override {
		return fns.ShouldCollide(self, inLayer1, to_jpc(inLayer2));
	}

private:
	const void* self;
	JPC_ObjectVsBroadPhaseLayerFilterFns fns;
};

OPAQUE_WRAPPER(JPC_ObjectVsBroadPhaseLayerFilter, JPC_ObjectVsBroadPhaseLayerFilterBridge)
DESTRUCTOR(JPC_ObjectVsBroadPhaseLayerFilter)

JPC_API JPC_ObjectVsBroadPhaseLayerFilter* JPC_ObjectVsBroadPhaseLayerFilter_new(
	const void *self,
	JPC_ObjectVsBroadPhaseLayerFilterFns fns)
{
	return to_jpc(new JPC_ObjectVsBroadPhaseLayerFilterBridge(self, fns));
}

////////////////////////////////////////////////////////////////////////////////
// BroadPhaseLayerFilter

class JPC_BroadPhaseLayerFilterBridge final : public JPH::BroadPhaseLayerFilter {
public:
	explicit JPC_BroadPhaseLayerFilterBridge(const void *self, JPC_BroadPhaseLayerFilterFns fns) : self(self), fns(fns) {}

	virtual bool ShouldCollide(JPH::BroadPhaseLayer inLayer) const override {
		return fns.ShouldCollide(self, to_jpc(inLayer));
	}

private:
	const void* self;
	JPC_BroadPhaseLayerFilterFns fns;
};

OPAQUE_WRAPPER(JPC_BroadPhaseLayerFilter, JPC_BroadPhaseLayerFilterBridge)
DESTRUCTOR(JPC_BroadPhaseLayerFilter)

JPC_API JPC_BroadPhaseLayerFilter* JPC_BroadPhaseLayerFilter_new(
	const void *self,
	JPC_BroadPhaseLayerFilterFns fns)
{
	return to_jpc(new JPC_BroadPhaseLayerFilterBridge(self, fns));
}

////////////////////////////////////////////////////////////////////////////////
// ObjectLayerFilter

class JPC_ObjectLayerFilterBridge final : public JPH::ObjectLayerFilter {
public:
	explicit JPC_ObjectLayerFilterBridge(const void *self, JPC_ObjectLayerFilterFns fns) : self(self), fns(fns) {}

	virtual bool ShouldCollide(JPH::ObjectLayer inLayer) const override {
		return fns.ShouldCollide(self, inLayer);
	}

private:
	const void* self;
	JPC_ObjectLayerFilterFns fns;
};

OPAQUE_WRAPPER(JPC_ObjectLayerFilter, JPC_ObjectLayerFilterBridge)
DESTRUCTOR(JPC_ObjectLayerFilter)

JPC_API JPC_ObjectLayerFilter* JPC_ObjectLayerFilter_new(
	const void *self,
	JPC_ObjectLayerFilterFns fns)
{
	return to_jpc(new JPC_ObjectLayerFilterBridge(self, fns));
}

////////////////////////////////////////////////////////////////////////////////
// BodyFilter

class JPC_BodyFilterBridge final : public JPH::BodyFilter {
public:
	explicit JPC_BodyFilterBridge(const void *self, JPC_BodyFilterFns fns) : self(self), fns(fns) {}

	virtual bool ShouldCollide(const JPH::BodyID &inBodyID) const override {
		return fns.ShouldCollide(self, to_jpc(inBodyID));
	}

	virtual bool ShouldCollideLocked(const JPH::Body &inBody) const override {
		return fns.ShouldCollideLocked(self, to_jpc(&inBody));
	}

private:
	const void* self;
	JPC_BodyFilterFns fns;
};

OPAQUE_WRAPPER(JPC_BodyFilter, JPC_BodyFilterBridge)
DESTRUCTOR(JPC_BodyFilter)

JPC_API JPC_BodyFilter* JPC_BodyFilter_new(
	const void *self,
	JPC_BodyFilterFns fns)
{
	return to_jpc(new JPC_BodyFilterBridge(self, fns));
}

////////////////////////////////////////////////////////////////////////////////
// ShapeFilter

class JPC_ShapeFilterBridge final : public JPH::ShapeFilter {
public:
	explicit JPC_ShapeFilterBridge(const void *self, JPC_ShapeFilterFns fns) : self(self), fns(fns) {}

	virtual bool ShouldCollide(const JPH::Shape *inShape2, const JPH::SubShapeID &inSubShapeIDOfShape2) const override {
		if (fns.ShouldCollide == nullptr) {
			return true;
		}

		return fns.ShouldCollide(self, to_jpc(inShape2), to_jpc(inSubShapeIDOfShape2));
	}

	virtual bool ShouldCollide(
		const JPH::Shape *inShape1, const JPH::SubShapeID &inSubShapeIDOfShape1,
		const JPH::Shape *inShape2, const JPH::SubShapeID &inSubShapeIDOfShape2) const override
	{
		if (fns.ShouldCollideTwoShapes == nullptr) {
			return true;
		}

		return fns.ShouldCollideTwoShapes(self,
			to_jpc(inShape1), to_jpc(inSubShapeIDOfShape1),
			to_jpc(inShape2), to_jpc(inSubShapeIDOfShape2));
	}

private:
	const void* self;
	JPC_ShapeFilterFns fns;
};

OPAQUE_WRAPPER(JPC_ShapeFilter, JPC_ShapeFilterBridge)
DESTRUCTOR(JPC_ShapeFilter)

JPC_API JPC_ShapeFilter* JPC_ShapeFilter_new(
	const void *self,
	JPC_ShapeFilterFns fns)
{
	return to_jpc(new JPC_ShapeFilterBridge(self, fns));
}

////////////////////////////////////////////////////////////////////////////////
// SimShapeFilter

class JPC_SimShapeFilterBridge final : public JPH::SimShapeFilter {
public:
	explicit JPC_SimShapeFilterBridge(const void *self, JPC_SimShapeFilterFns fns) : self(self), fns(fns) {}

	virtual bool ShouldCollide(
		const JPH::Body &inBody1, const JPH::Shape *inShape1, const JPH::SubShapeID &inSubShapeIDOfShape1,
		const JPH::Body &inBody2, const JPH::Shape *inShape2, const JPH::SubShapeID &inSubShapeIDOfShape2) const override
	{
		if (fns.ShouldCollide == nullptr) {
			return true;
		}

		return fns.ShouldCollide(self,
			to_jpc(&inBody1), to_jpc(inShape1), to_jpc(inSubShapeIDOfShape1),
			to_jpc(&inBody2), to_jpc(inShape2), to_jpc(inSubShapeIDOfShape2));
	}

private:
	const void* self;
	JPC_SimShapeFilterFns fns;
};

OPAQUE_WRAPPER(JPC_SimShapeFilter, JPC_SimShapeFilterBridge)
DESTRUCTOR(JPC_SimShapeFilter)

JPC_API JPC_SimShapeFilter* JPC_SimShapeFilter_new(
	const void *self,
	JPC_SimShapeFilterFns fns)
{
	return to_jpc(new JPC_SimShapeFilterBridge(self, fns));
}

////////////////////////////////////////////////////////////////////////////////
// JPC_ObjectLayerPairFilter

class JPC_ObjectLayerPairFilterBridge final : public JPH::ObjectLayerPairFilter {
public:
	explicit JPC_ObjectLayerPairFilterBridge(const void *self, JPC_ObjectLayerPairFilterFns fns) : self(self), fns(fns) {}

	virtual bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::ObjectLayer inLayer2) const override {
		return fns.ShouldCollide(self, inLayer1, inLayer2);
	}

private:
	const void* self;
	JPC_ObjectLayerPairFilterFns fns;
};

OPAQUE_WRAPPER(JPC_ObjectLayerPairFilter, JPC_ObjectLayerPairFilterBridge)
DESTRUCTOR(JPC_ObjectLayerPairFilter)

JPC_API JPC_ObjectLayerPairFilter* JPC_ObjectLayerPairFilter_new(
	const void *self,
	JPC_ObjectLayerPairFilterFns fns)
{
	return to_jpc(new JPC_ObjectLayerPairFilterBridge(self, fns));
}

////////////////////////////////////////////////////////////////////////////////
// JPC_ContactListener

class JPC_ContactListenerBridge final : public JPH::ContactListener {
public:
	explicit JPC_ContactListenerBridge(void *self, JPC_ContactListenerFns fns) : self(self), fns(fns) {}

	JPH::ValidateResult OnContactValidate(
		const JPH::Body &inBody1,
		const JPH::Body &inBody2,
		JPH::RVec3Arg inBaseOffset,
		const JPH::CollideShapeResult &inCollisionResult) override
	{
		if (fns.OnContactValidate != nullptr) {
			JPC_CollideShapeResult collisionResult = JPC_CollideShapeResult_to_jpc(inCollisionResult);
			return to_jph(fns.OnContactValidate(self, to_jpc(&inBody1), to_jpc(&inBody2), to_jpc(inBaseOffset), &collisionResult));
		}
		return ContactListener::OnContactValidate(inBody1, inBody2, inBaseOffset, inCollisionResult);
	}

	void OnContactAdded(
		const JPH::Body &inBody1,
		const JPH::Body &inBody2,
		const JPH::ContactManifold &inManifold,
		JPH::ContactSettings &ioSettings) override
	{
		if (fns.OnContactAdded != nullptr) {
			const auto* cManifold = reinterpret_cast<const JPC_ContactManifold*>(&inManifold);
			auto* cSettings = reinterpret_cast<JPC_ContactSettings*>(&ioSettings);

			fns.OnContactAdded(self, to_jpc(&inBody1), to_jpc(&inBody2), cManifold, cSettings);
		}
	}

	void OnContactPersisted(
		const JPH::Body &inBody1,
		const JPH::Body &inBody2,
		const JPH::ContactManifold &inManifold,
		JPH::ContactSettings &ioSettings) override
	{
		if (fns.OnContactPersisted != nullptr) {
			const auto* cManifold = reinterpret_cast<const JPC_ContactManifold*>(&inManifold);
			auto* cSettings = reinterpret_cast<JPC_ContactSettings*>(&ioSettings);

			fns.OnContactPersisted(self, to_jpc(&inBody1), to_jpc(&inBody2), cManifold, cSettings);
		}
	}

	void OnContactRemoved(const JPH::SubShapeIDPair &inSubShapePair) override {
		if (fns.OnContactRemoved != nullptr) {
			const auto* cSubShapePair = reinterpret_cast<const JPC_SubShapeIDPair*>(&inSubShapePair);

			fns.OnContactRemoved(self, cSubShapePair);
		}
	}

private:
	void* self;
	JPC_ContactListenerFns fns;
};

OPAQUE_WRAPPER(JPC_ContactListener, JPC_ContactListenerBridge)
DESTRUCTOR(JPC_ContactListener)

JPC_API JPC_ContactListener* JPC_ContactListener_new(
	void *self,
	JPC_ContactListenerFns fns)
{
	return to_jpc(new JPC_ContactListenerBridge(self, fns));
}

JPC_API void JPC_EstimateCollisionResponse(
	const JPC_Body* inBody1,
	const JPC_Body* inBody2,
	const JPC_ContactManifold* inManifold,
	JPC_CollisionEstimationResult* outResult,
	float inCombinedFriction,
	float inCombinedRestitution,
	float inMinVelocityForRestitution,	///< = 1.0f
	uint inNumIterations				///< = 10
) {
	const auto* jphManifold = reinterpret_cast<const JPH::ContactManifold*>(inManifold);
	auto* jphResult = reinterpret_cast<JPH::CollisionEstimationResult*>(outResult);

	JPH::EstimateCollisionResponse(
		*to_jph(inBody1),
		*to_jph(inBody2),
		*jphManifold,
		*jphResult,
		inCombinedFriction,
		inCombinedRestitution,
		inMinVelocityForRestitution,
		inNumIterations);
}

////////////////////////////////////////////////////////////////////////////////
// JPC_CastShapeCollector

class JPC_CastShapeCollectorBridge;
OPAQUE_WRAPPER(JPC_CastShapeCollector, JPC_CastShapeCollectorBridge)

class JPC_CastShapeCollectorBridge final : public JPH::CastShapeCollector {
	using ResultType = JPH::ShapeCastResult;

public:
	explicit JPC_CastShapeCollectorBridge(void *self, JPC_CastShapeCollectorFns fns) : self(self), fns(fns) {}

	void Reset() override {
		JPH::CastShapeCollector::Reset();

		if (fns.Reset != nullptr) {
			fns.Reset(self);
		}
	}

	void AddHit(const ResultType &inResult) override {
		JPC_ShapeCastResult result = JPC_ShapeCastResult_to_jpc(inResult);
		JPC_CastShapeCollector *base = to_jpc(this);

		fns.AddHit(self, base, &result);
	}

private:
	void* self;
	JPC_CastShapeCollectorFns fns;
};

DESTRUCTOR(JPC_CastShapeCollector)

JPC_API JPC_CastShapeCollector* JPC_CastShapeCollector_new(
	void *self,
	JPC_CastShapeCollectorFns fns)
{
	return to_jpc(new JPC_CastShapeCollectorBridge(self, fns));
}

JPC_API void JPC_CastShapeCollector_UpdateEarlyOutFraction(JPC_CastShapeCollector* self, float inFraction) {
	to_jph(self)->UpdateEarlyOutFraction(inFraction);
}

////////////////////////////////////////////////////////////////////////////////
// JPC_CollideShapeCollector

class JPC_CollideShapeCollectorBridge;
OPAQUE_WRAPPER(JPC_CollideShapeCollector, JPC_CollideShapeCollectorBridge)

class JPC_CollideShapeCollectorBridge final : public JPH::CollideShapeCollector {
	using ResultType = JPH::CollideShapeResult;

public:
	explicit JPC_CollideShapeCollectorBridge(void *self, JPC_CollideShapeCollectorFns fns) : self(self), fns(fns) {}

	void Reset() override {
		JPH::CollideShapeCollector::Reset();

		if (fns.Reset != nullptr) {
			fns.Reset(self);
		}
	}

	void AddHit(const ResultType &inResult) override {
		JPC_CollideShapeResult result = JPC_CollideShapeResult_to_jpc(inResult);
		JPC_CollideShapeCollector *base = to_jpc(this);

		fns.AddHit(self, base, &result);
	}

private:
	void* self;
	JPC_CollideShapeCollectorFns fns;
};

DESTRUCTOR(JPC_CollideShapeCollector)

JPC_API JPC_CollideShapeCollector* JPC_CollideShapeCollector_new(
	void *self,
	JPC_CollideShapeCollectorFns fns)
{
	return to_jpc(new JPC_CollideShapeCollectorBridge(self, fns));
}

JPC_API void JPC_CollideShapeCollector_UpdateEarlyOutFraction(JPC_CollideShapeCollector* self, float inFraction) {
	to_jph(self)->UpdateEarlyOutFraction(inFraction);
}

////////////////////////////////////////////////////////////////////////////////
// BodyManager::DrawSettings

JPC_API void JPC_BodyManager_DrawSettings_default(JPC_BodyManager_DrawSettings* object) {
	*object = to_jpc(JPH::BodyManager::DrawSettings());
}

////////////////////////////////////////////////////////////////////////////////
// DebugRendererSimple

class JPC_DebugRendererSimpleBridge final : public JPH::DebugRendererSimple {
public:
	explicit JPC_DebugRendererSimpleBridge(const void *self, JPC_DebugRendererSimpleFns fns) : self(self), fns(fns) {}

	virtual void DrawLine(JPH::RVec3Arg inFrom, JPH::RVec3Arg inTo, JPH::ColorArg inColor) override {
		fns.DrawLine(self, to_jpc(inFrom), to_jpc(inTo), to_jpc(inColor));
	}

	virtual void DrawText3D(
		[[maybe_unused]] JPH::RVec3Arg inPosition,
		[[maybe_unused]] const std::string_view &inString,
		[[maybe_unused]] JPH::ColorArg inColor = JPH::Color::sWhite,
		[[maybe_unused]] float inHeight = 0.5f) override
	{
		// TODO
	}

private:
	const void* self;
	JPC_DebugRendererSimpleFns fns;
};

OPAQUE_WRAPPER(JPC_DebugRendererSimple, JPC_DebugRendererSimpleBridge)
DESTRUCTOR(JPC_DebugRendererSimple)

JPC_API JPC_DebugRendererSimple* JPC_DebugRendererSimple_new(
	const void *self,
	JPC_DebugRendererSimpleFns fns)
{
	return to_jpc(new JPC_DebugRendererSimpleBridge(self, fns));
}

////////////////////////////////////////////////////////////////////////////////
// String

JPC_API const char* JPC_String_c_str(JPC_String* self) {
	return to_jph(self)->c_str();
}

////////////////////////////////////////////////////////////////////////////////
// Constraint -> RefTarget<Constraint>

OPAQUE_WRAPPER(JPC_Constraint, JPH::Constraint);

// RefTarget<Constraint>
JPC_API uint32_t JPC_Constraint_GetRefCount(const JPC_Constraint* self) {
	return to_jph(self)->GetRefCount();
}

JPC_API void JPC_Constraint_AddRef(const JPC_Constraint* self) {
	to_jph(self)->AddRef();
}

JPC_API void JPC_Constraint_Release(const JPC_Constraint* self) {
	to_jph(self)->Release();
}

// Constraint
JPC_API void JPC_Constraint_delete(JPC_Constraint* self) {
	delete to_jph(self);
}

JPC_API JPC_ConstraintType JPC_Constraint_GetType(const JPC_Constraint* self) {
	return (JPC_ConstraintType)to_jph(self)->GetType();
}

JPC_API JPC_ConstraintSubType JPC_Constraint_GetSubType(const JPC_Constraint* self) {
	return (JPC_ConstraintSubType)to_jph(self)->GetSubType();
}

JPC_API uint32_t JPC_Constraint_GetConstraintPriority(const JPC_Constraint* self) {
	return to_jph(self)->GetConstraintPriority();
}

JPC_API void JPC_Constraint_SetConstraintPriority(JPC_Constraint* self, uint32_t inPriority) {
	to_jph(self)->SetConstraintPriority(inPriority);
}

JPC_API uint JPC_Constraint_GetNumVelocityStepsOverride(const JPC_Constraint* self) {
	return to_jph(self)->GetNumVelocityStepsOverride();
}

JPC_API void JPC_Constraint_SetNumVelocityStepsOverride(JPC_Constraint* self, uint inN) {
	to_jph(self)->SetNumVelocityStepsOverride(inN);
}

JPC_API uint JPC_Constraint_GetNumPositionStepsOverride(const JPC_Constraint* self) {
	return to_jph(self)->GetNumPositionStepsOverride();
}

JPC_API void JPC_Constraint_SetNumPositionStepsOverride(JPC_Constraint* self, uint inN) {
	to_jph(self)->SetNumPositionStepsOverride(inN);
}

JPC_API bool JPC_Constraint_GetEnabled(const JPC_Constraint* self) {
	return to_jph(self)->GetEnabled();
}

JPC_API void JPC_Constraint_SetEnabled(JPC_Constraint* self, bool inEnabled) {
	to_jph(self)->SetEnabled(inEnabled);
}

JPC_API uint64_t JPC_Constraint_GetUserData(const JPC_Constraint* self) {
	return to_jph(self)->GetUserData();
}

JPC_API void JPC_Constraint_SetUserData(JPC_Constraint* self, uint64_t inUserData) {
	to_jph(self)->SetUserData(inUserData);
}

JPC_API void JPC_Constraint_NotifyShapeChanged(JPC_Constraint* self, JPC_BodyID inBodyID, JPC_Vec3 inDeltaCOM) {
	to_jph(self)->NotifyShapeChanged(to_jph(inBodyID), to_jph(inDeltaCOM));
}

////////////////////////////////////////////////////////////////////////////////
// TwoBodyConstraint -> Constraint -> RefTarget<Constraint>

OPAQUE_WRAPPER(JPC_TwoBodyConstraint, JPH::TwoBodyConstraint);

JPC_API JPC_Body* JPC_TwoBodyConstraint_GetBody1(const JPC_TwoBodyConstraint* self) {
	return to_jpc(to_jph(self)->GetBody1());
}

JPC_API JPC_Body* JPC_TwoBodyConstraint_GetBody2(const JPC_TwoBodyConstraint* self) {
	return to_jpc(to_jph(self)->GetBody2());
}

JPC_API JPC_Mat44 JPC_TwoBodyConstraint_GetConstraintToBody1Matrix(const JPC_TwoBodyConstraint* self) {
	return to_jpc(to_jph(self)->GetConstraintToBody1Matrix());
}

JPC_API JPC_Mat44 JPC_TwoBodyConstraint_GetConstraintToBody2Matrix(const JPC_TwoBodyConstraint* self) {
	return to_jpc(to_jph(self)->GetConstraintToBody2Matrix());
}

////////////////////////////////////////////////////////////////////////////////
// FixedConstraint -> TwoBodyConstraint -> Constraint -> RefTarget<Constraint>

OPAQUE_WRAPPER(JPC_FixedConstraint, JPH::FixedConstraint);

JPC_API JPC_Vec3 JPC_FixedConstraint_GetTotalLambdaPosition(const JPC_FixedConstraint* self) {
	return to_jpc(to_jph(self)->GetTotalLambdaPosition());
}

JPC_API JPC_Vec3 JPC_FixedConstraint_GetTotalLambdaRotation(const JPC_FixedConstraint* self) {
	return to_jpc(to_jph(self)->GetTotalLambdaRotation());
}

////////////////////////////////////////////////////////////////////////////////
// DistanceConstraint -> TwoBodyConstraint -> Constraint -> RefTarget<Constraint>

OPAQUE_WRAPPER(JPC_DistanceConstraint, JPH::DistanceConstraint);

JPC_API float JPC_DistanceConstraint_GetTotalLambdaPosition(const JPC_DistanceConstraint* self) {
	return to_jph(self)->GetTotalLambdaPosition();
}

////////////////////////////////////////////////////////////////////////////////
// SixDOFConstraint -> TwoBodyConstraint -> Constraint -> RefTarget<Constraint>

OPAQUE_WRAPPER(JPC_SixDOFConstraint, JPH::SixDOFConstraint);

JPC_API JPC_Vec3 JPC_SixDOFConstraint_GetTranslationLimitsMin(const JPC_SixDOFConstraint* self) {
	return to_jpc(to_jph(self)->GetTranslationLimitsMin());
}

JPC_API JPC_Vec3 JPC_SixDOFConstraint_GetTranslationLimitsMax(const JPC_SixDOFConstraint* self) {
	return to_jpc(to_jph(self)->GetTranslationLimitsMax());
}

JPC_API void JPC_SixDOFConstraint_SetTranslationLimits(JPC_SixDOFConstraint* self, JPC_Vec3 inLimitMin, JPC_Vec3 inLimitMax) {
	to_jph(self)->SetTranslationLimits(to_jph(inLimitMin), to_jph(inLimitMax));
}

JPC_API JPC_Vec3 JPC_SixDOFConstraint_GetRotationLimitsMin(const JPC_SixDOFConstraint* self) {
	return to_jpc(to_jph(self)->GetRotationLimitsMin());
}

JPC_API JPC_Vec3 JPC_SixDOFConstraint_GetRotationLimitsMax(const JPC_SixDOFConstraint* self) {
	return to_jpc(to_jph(self)->GetRotationLimitsMax());
}

JPC_API void JPC_SixDOFConstraint_SetRotationLimits(JPC_SixDOFConstraint* self, JPC_Vec3 inLimitMin, JPC_Vec3 inLimitMax) {
	to_jph(self)->SetRotationLimits(to_jph(inLimitMin), to_jph(inLimitMax));
}

JPC_API float JPC_SixDOFConstraint_GetLimitsMin(const JPC_SixDOFConstraint* self, JPC_SixDOFConstraint_Axis inAxis) {
	return to_jph(self)->GetLimitsMin((JPH::SixDOFConstraint::EAxis)inAxis);
}

JPC_API float JPC_SixDOFConstraint_GetLimitsMax(const JPC_SixDOFConstraint* self, JPC_SixDOFConstraint_Axis inAxis) {
	return to_jph(self)->GetLimitsMax((JPH::SixDOFConstraint::EAxis)inAxis);
}

JPC_API bool JPC_SixDOFConstraint_IsFreeAxis(const JPC_SixDOFConstraint* self, JPC_SixDOFConstraint_Axis inAxis);

// const SpringSettings & GetLimitsSpringSettings(JPC_SixDOFConstraint_Axis inAxis) const { JPH_ASSERT(inAxis < JPC_SixDOFConstraint_Axis::NumTranslation); return mLimitsSpringSettings[inAxis]; }
// void SetLimitsSpringSettings(JPC_SixDOFConstraint_Axis inAxis, const SpringSettings& inLimitsSpringSettings) { JPH_ASSERT(inAxis < JPC_SixDOFConstraint_Axis::NumTranslation); mLimitsSpringSettings[inAxis] = inLimitsSpringSettings; CacheHasSpringLimits(); }

JPC_API void JPC_SixDOFConstraint_SetMaxFriction(JPC_SixDOFConstraint* self, JPC_SixDOFConstraint_Axis inAxis, float inFriction) {
	to_jph(self)->SetMaxFriction((JPH::SixDOFConstraint::EAxis)inAxis, inFriction);
}

JPC_API float JPC_SixDOFConstraint_GetMaxFriction(const JPC_SixDOFConstraint* self, JPC_SixDOFConstraint_Axis inAxis) {
	return to_jph(self)->GetMaxFriction((JPH::SixDOFConstraint::EAxis)inAxis);
}

JPC_API JPC_Quat JPC_SixDOFConstraint_GetRotationInConstraintSpace(const JPC_SixDOFConstraint* self) {
	return to_jpc(to_jph(self)->GetRotationInConstraintSpace());
}

/// Motor settings
// MotorSettings & GetMotorSettings(EAxis inAxis)
// const MotorSettings & GetMotorSettings(EAxis inAxis) const

// void SetMotorState(EAxis inAxis, EMotorState inState);
// EMotorState GetMotorState(EAxis inAxis) const

JPC_API JPC_Vec3 JPC_SixDOFConstraint_GetTargetVelocityCS(const JPC_SixDOFConstraint* self) {
	return to_jpc(to_jph(self)->GetTargetVelocityCS());
}

JPC_API void JPC_SixDOFConstraint_SetTargetVelocityCS(JPC_SixDOFConstraint* self, JPC_Vec3 inVelocity) {
	to_jph(self)->SetTargetVelocityCS(to_jph(inVelocity));
}

JPC_API JPC_Vec3 JPC_SixDOFConstraint_GetTargetAngularVelocityCS(const JPC_SixDOFConstraint* self) {
	return to_jpc(to_jph(self)->GetTargetAngularVelocityCS());
}

JPC_API void JPC_SixDOFConstraint_SetTargetAngularVelocityCS(JPC_SixDOFConstraint* self, JPC_Vec3 inAngularVelocity) {
	to_jph(self)->SetTargetAngularVelocityCS(to_jph(inAngularVelocity));
}

JPC_API JPC_Vec3 JPC_SixDOFConstraint_GetTargetPositionCS(const JPC_SixDOFConstraint* self) {
	return to_jpc(to_jph(self)->GetTargetPositionCS());
}

JPC_API void JPC_SixDOFConstraint_SetTargetPositionCS(JPC_SixDOFConstraint* self, JPC_Vec3 inPosition) {
	to_jph(self)->SetTargetPositionCS(to_jph(inPosition));
}

JPC_API JPC_Quat JPC_SixDOFConstraint_GetTargetOrientationCS(const JPC_SixDOFConstraint* self) {
	return to_jpc(to_jph(self)->GetTargetOrientationCS());
}

JPC_API void JPC_SixDOFConstraint_SetTargetOrientationCS(JPC_SixDOFConstraint* self, JPC_Quat inOrientation) {
	to_jph(self)->SetTargetOrientationCS(to_jph(inOrientation));
}

JPC_API void JPC_SixDOFConstraint_SetTargetOrientationBS(JPC_SixDOFConstraint* self, JPC_Quat inOrientation) {
	to_jph(self)->SetTargetOrientationBS(to_jph(inOrientation));
}

JPC_API JPC_Vec3 JPC_SixDOFConstraint_GetTotalLambdaPosition(JPC_SixDOFConstraint* self) {
	return to_jpc(to_jph(self)->GetTotalLambdaPosition());
}

JPC_API JPC_Vec3 JPC_SixDOFConstraint_GetTotalLambdaRotation(JPC_SixDOFConstraint* self) {
	return to_jpc(to_jph(self)->GetTotalLambdaRotation());
}

JPC_API JPC_Vec3 JPC_SixDOFConstraint_GetTotalLambdaMotorTranslation(JPC_SixDOFConstraint* self) {
	return to_jpc(to_jph(self)->GetTotalLambdaMotorTranslation());
}

JPC_API JPC_Vec3 JPC_SixDOFConstraint_GetTotalLambdaMotorRotation(JPC_SixDOFConstraint* self) {
	return to_jpc(to_jph(self)->GetTotalLambdaMotorRotation());
}

////////////////////////////////////////////////////////////////////////////////
// HingeConstraint -> TwoBodyConstraint -> Constraint -> RefTarget<Constraint>

OPAQUE_WRAPPER(JPC_HingeConstraint, JPH::HingeConstraint);

JPC_API JPC_Constraint* JPC_HingeConstraint_to_Constraint(JPC_HingeConstraint* self) {
	return (JPC_Constraint*)(self);
}

JPC_API void JPC_HingeConstraint_SetMotorState(JPC_HingeConstraint* self, JPC_MotorState inState) {
	to_jph(self)->SetMotorState(to_jph(inState));
}

JPC_API JPC_MotorState JPC_HingeConstraint_GetMotorState(const JPC_HingeConstraint* self) {
	return to_jpc(to_jph(self)->GetMotorState());
}

JPC_API void JPC_HingeConstraint_SetTargetAngularVelocity(JPC_HingeConstraint* self, float inAngularVelocity) {
	to_jph(self)->SetTargetAngularVelocity(inAngularVelocity);
}

JPC_API float JPC_HingeConstraint_GetTargetAngularVelocity(const JPC_HingeConstraint* self) {
	return to_jph(self)->GetTargetAngularVelocity();
}

JPC_API void JPC_HingeConstraint_SetTargetAngle(JPC_HingeConstraint* self, float inAngle) {
	to_jph(self)->SetTargetAngle(inAngle);
}

JPC_API float JPC_HingeConstraint_GetTargetAngle(const JPC_HingeConstraint* self) {
	return to_jph(self)->GetTargetAngle();
}

JPC_API JPC_Vec3 JPC_HingeConstraint_GetTotalLambdaPosition(const JPC_HingeConstraint* self) {
	return to_jpc(to_jph(self)->GetTotalLambdaPosition());
}

JPC_API JPC_Vec2 JPC_HingeConstraint_GetTotalLambdaRotation(const JPC_HingeConstraint* self) {
	return to_jpc(to_jph(self)->GetTotalLambdaRotation());
}

JPC_API float JPC_HingeConstraint_GetTotalLambdaRotationLimits(const JPC_HingeConstraint* self) {
	return to_jph(self)->GetTotalLambdaRotationLimits();
}

JPC_API float JPC_HingeConstraint_GetTotalLambdaMotor(const JPC_HingeConstraint* self) {
	return to_jph(self)->GetTotalLambdaMotor();
}


////////////////////////////////////////////////////////////////////////////////
// SliderConstraint -> TwoBodyConstraint -> Constraint -> RefTarget<Constraint>

OPAQUE_WRAPPER(JPC_SliderConstraint, JPH::SliderConstraint);

JPC_API void JPC_SliderConstraint_SetMotorState(JPC_SliderConstraint* self, JPC_MotorState inState) {
	to_jph(self)->SetMotorState(to_jph(inState));
}

JPC_API JPC_MotorState JPC_SliderConstraint_GetMotorState(const JPC_SliderConstraint* self) {
	return to_jpc(to_jph(self)->GetMotorState());
}

JPC_API void JPC_SliderConstraint_SetTargetVelocity(JPC_SliderConstraint* self, float inVelocity) {
	to_jph(self)->SetTargetVelocity(inVelocity);
}

JPC_API float JPC_SliderConstraint_GetTargetVelocity(const JPC_SliderConstraint* self) {
	return to_jph(self)->GetTargetVelocity();
}

JPC_API void JPC_SliderConstraint_SetTargetPosition(JPC_SliderConstraint* self, float inPosition) {
	to_jph(self)->SetTargetPosition(inPosition);
}

JPC_API float JPC_SliderConstraint_GetTargetPosition(const JPC_SliderConstraint* self) {
	return to_jph(self)->GetTargetPosition();
}

JPC_API JPC_Vec2 JPC_SliderConstraint_GetTotalLambdaPosition(const JPC_SliderConstraint* self) {
	return to_jpc(to_jph(self)->GetTotalLambdaPosition());
}

JPC_API float JPC_SliderConstraint_GetTotalLambdaPositionLimits(const JPC_SliderConstraint* self) {
	return to_jph(self)->GetTotalLambdaPositionLimits();
}

JPC_API JPC_Vec3 JPC_SliderConstraint_GetTotalLambdaRotation(const JPC_SliderConstraint* self) {
	return to_jpc(to_jph(self)->GetTotalLambdaRotation());
}

JPC_API float JPC_SliderConstraint_GetTotalLambdaMotor(const JPC_SliderConstraint* self) {
	return to_jph(self)->GetTotalLambdaMotor();
}


////////////////////////////////////////////////////////////////////////////////
// ConstraintSettings

JPC_IMPL void JPC_ConstraintSettings_to_jpc(
	JPC_ConstraintSettings* outJpc,
	const JPH::ConstraintSettings* inJph)
{
	outJpc->Enabled = inJph->mEnabled;
	outJpc->ConstraintPriority = inJph->mConstraintPriority;
	outJpc->NumVelocityStepsOverride = inJph->mNumVelocityStepsOverride;
	outJpc->NumPositionStepsOverride = inJph->mNumPositionStepsOverride;
	outJpc->DrawConstraintSize = inJph->mDrawConstraintSize;
	outJpc->UserData = inJph->mUserData;
}

JPC_IMPL void JPC_ConstraintSettings_to_jph(
	const JPC_ConstraintSettings* inJpc,
	JPH::ConstraintSettings* outJph)
{
	outJph->mEnabled = inJpc->Enabled;
	outJph->mConstraintPriority = inJpc->ConstraintPriority;
	outJph->mNumVelocityStepsOverride = inJpc->NumVelocityStepsOverride;
	outJph->mNumPositionStepsOverride = inJpc->NumPositionStepsOverride;
	outJph->mDrawConstraintSize = inJpc->DrawConstraintSize;
	outJph->mUserData = inJpc->UserData;
}

JPC_API void JPC_ConstraintSettings_default(JPC_ConstraintSettings* settings) {
	// ConstraintSettings default constructor is protected in 5.5.0+, so we
	// hardcode the defaults from Constraint.h directly.
	settings->Enabled = true;
	settings->ConstraintPriority = 0;
	settings->NumVelocityStepsOverride = 0;
	settings->NumPositionStepsOverride = 0;
	settings->DrawConstraintSize = 1.0f;
	settings->UserData = 0;
}

////////////////////////////////////////////////////////////////////////////////
// SpringSettings

JPC_IMPL void JPC_SpringSettings_to_jpc(
	JPC_SpringSettings* outJpc,
	const JPH::SpringSettings* inJph)
{
	outJpc->Mode = to_jpc(inJph->mMode);
	outJpc->FrequencyOrStiffness = inJph->mFrequency;
	outJpc->Damping = inJph->mDamping;
}

JPC_IMPL void JPC_SpringSettings_to_jph(
	const JPC_SpringSettings* inJpc,
	JPH::SpringSettings* outJph)
{
	outJph->mMode = to_jph(inJpc->Mode);
	outJph->mFrequency = inJpc->FrequencyOrStiffness;
	outJph->mDamping = inJpc->Damping;
}

JPC_API void JPC_SpringSettings_default(JPC_SpringSettings* settings) {
	JPH::SpringSettings defaultSettings{};
	JPC_SpringSettings_to_jpc(settings, &defaultSettings);
}

////////////////////////////////////////////////////////////////////////////////
// MotorSettings

JPC_IMPL void JPC_MotorSettings_to_jpc(
	JPC_MotorSettings* outJpc,
	const JPH::MotorSettings* inJph)
{
	JPC_SpringSettings_to_jpc(&outJpc->SpringSettings, &inJph->mSpringSettings);
	outJpc->MinForceLimit = inJph->mMinForceLimit;
	outJpc->MaxForceLimit = inJph->mMaxForceLimit;
	outJpc->MinTorqueLimit = inJph->mMinTorqueLimit;
	outJpc->MaxTorqueLimit = inJph->mMaxTorqueLimit;
}

JPC_IMPL void JPC_MotorSettings_to_jph(
	const JPC_MotorSettings* inJpc,
	JPH::MotorSettings* outJph)
{
	JPC_SpringSettings_to_jph(&inJpc->SpringSettings, &outJph->mSpringSettings);
	outJph->mMinForceLimit = inJpc->MinForceLimit;
	outJph->mMaxForceLimit = inJpc->MaxForceLimit;
	outJph->mMinTorqueLimit = inJpc->MinTorqueLimit;
	outJph->mMaxTorqueLimit = inJpc->MaxTorqueLimit;
}

JPC_API void JPC_MotorSettings_default(JPC_MotorSettings* settings) {
	JPH::MotorSettings defaultSettings{};
	JPC_MotorSettings_to_jpc(settings, &defaultSettings);
}

////////////////////////////////////////////////////////////////////////////////
// FixedConstraintSettings -> TwoBodyConstraintSettings -> ConstraintSettings

JPC_IMPL void JPC_FixedConstraintSettings_to_jpc(
	JPC_FixedConstraintSettings* outJpc,
	const JPH::FixedConstraintSettings* inJph)
{
	JPC_ConstraintSettings_to_jpc(&outJpc->ConstraintSettings, inJph);

	outJpc->Space = static_cast<JPC_ConstraintSpace>(inJph->mSpace);
	outJpc->AutoDetectPoint = inJph->mAutoDetectPoint;
	outJpc->Point1 = to_jpc(inJph->mPoint1);
	outJpc->AxisX1 = to_jpc(inJph->mAxisX1);
	outJpc->AxisY1 = to_jpc(inJph->mAxisY1);
	outJpc->Point2 = to_jpc(inJph->mPoint2);
	outJpc->AxisX2 = to_jpc(inJph->mAxisX2);
	outJpc->AxisY2 = to_jpc(inJph->mAxisY2);
}

JPC_IMPL void JPC_FixedConstraintSettings_to_jph(
	const JPC_FixedConstraintSettings* inJpc,
	JPH::FixedConstraintSettings* outJph)
{
	JPC_ConstraintSettings_to_jph(&inJpc->ConstraintSettings, outJph);

	outJph->mSpace = static_cast<JPH::EConstraintSpace>(inJpc->Space);
	outJph->mAutoDetectPoint = inJpc->AutoDetectPoint;
	outJph->mPoint1 = to_jph(inJpc->Point1);
	outJph->mAxisX1 = to_jph(inJpc->AxisX1);
	outJph->mAxisY1 = to_jph(inJpc->AxisY1);
	outJph->mPoint2 = to_jph(inJpc->Point2);
	outJph->mAxisX2 = to_jph(inJpc->AxisX2);
	outJph->mAxisY2 = to_jph(inJpc->AxisY2);
}

JPC_API void JPC_FixedConstraintSettings_default(JPC_FixedConstraintSettings* settings) {
	JPH::FixedConstraintSettings defaultSettings{};
	JPC_FixedConstraintSettings_to_jpc(settings, &defaultSettings);
}

JPC_API JPC_Constraint* JPC_FixedConstraintSettings_Create(
	const JPC_FixedConstraintSettings* self,
	JPC_Body* inBody1,
	JPC_Body* inBody2)
{
	JPH::FixedConstraintSettings jphSettings;
	JPC_FixedConstraintSettings_to_jph(self, &jphSettings);

	JPH::FixedConstraint* outJph = new JPH::FixedConstraint(*to_jph(inBody1), *to_jph(inBody2), jphSettings);
	return (JPC_Constraint*)outJph;
}

////////////////////////////////////////////////////////////////////////////////
// SixDOFConstraintSettings -> TwoBodyConstraintSettings -> ConstraintSettings

JPC_IMPL void JPC_SixDOFConstraintSettings_to_jpc(
	JPC_SixDOFConstraintSettings* outJpc,
	const JPH::SixDOFConstraintSettings* inJph)
{
	JPC_ConstraintSettings_to_jpc(&outJpc->ConstraintSettings, inJph);

	outJpc->Space = static_cast<JPC_ConstraintSpace>(inJph->mSpace);
	outJpc->Position1 = to_jpc(inJph->mPosition1);
	outJpc->AxisX1 = to_jpc(inJph->mAxisX1);
	outJpc->AxisY1 = to_jpc(inJph->mAxisY1);
	outJpc->Position2 = to_jpc(inJph->mPosition2);
	outJpc->AxisX2 = to_jpc(inJph->mAxisX2);
	outJpc->AxisY2 = to_jpc(inJph->mAxisY2);
	std::copy(inJph->mMaxFriction, inJph->mMaxFriction + 6, outJpc->MaxFriction);
	std::copy(inJph->mLimitMin, inJph->mLimitMin + 6, outJpc->LimitMin);
	std::copy(inJph->mLimitMax, inJph->mLimitMax + 6, outJpc->LimitMax);

	// TODO: LimitsSpringSettings
}

JPC_IMPL void JPC_SixDOFConstraintSettings_to_jph(
	const JPC_SixDOFConstraintSettings* inJpc,
	JPH::SixDOFConstraintSettings* outJph)
{
	JPC_ConstraintSettings_to_jph(&inJpc->ConstraintSettings, outJph);

	outJph->mSpace = static_cast<JPH::EConstraintSpace>(inJpc->Space);
	outJph->mPosition1 = to_jph(inJpc->Position1);
	outJph->mAxisX1 = to_jph(inJpc->AxisX1);
	outJph->mAxisY1 = to_jph(inJpc->AxisY1);
	outJph->mPosition2 = to_jph(inJpc->Position2);
	outJph->mAxisX2 = to_jph(inJpc->AxisX2);
	outJph->mAxisY2 = to_jph(inJpc->AxisY2);
	std::copy(inJpc->MaxFriction, inJpc->MaxFriction + 6, outJph->mMaxFriction);
	std::copy(inJpc->LimitMin, inJpc->LimitMin + 6, outJph->mLimitMin);
	std::copy(inJpc->LimitMax, inJpc->LimitMax + 6, outJph->mLimitMax);

	// TODO: LimitsSpringSettings
}

JPC_API void JPC_SixDOFConstraintSettings_default(JPC_SixDOFConstraintSettings* settings) {
	JPH::SixDOFConstraintSettings defaultSettings{};
	JPC_SixDOFConstraintSettings_to_jpc(settings, &defaultSettings);
}

JPC_API JPC_Constraint* JPC_SixDOFConstraintSettings_Create(
	const JPC_SixDOFConstraintSettings* self,
	JPC_Body* inBody1,
	JPC_Body* inBody2)
{
	JPH::SixDOFConstraintSettings jphSettings;
	JPC_SixDOFConstraintSettings_to_jph(self, &jphSettings);

	JPH::SixDOFConstraint* outJph = new JPH::SixDOFConstraint(*to_jph(inBody1), *to_jph(inBody2), jphSettings);
	return (JPC_Constraint*)outJph;
}

////////////////////////////////////////////////////////////////////////////////
// HingeConstraintSettings -> TwoBodyConstraintSettings -> ConstraintSettings

JPC_IMPL void JPC_HingeConstraintSettings_to_jpc(
	JPC_HingeConstraintSettings* outJpc,
	const JPH::HingeConstraintSettings* inJph)
{
	JPC_ConstraintSettings_to_jpc(&outJpc->ConstraintSettings, inJph);

	outJpc->Space = static_cast<JPC_ConstraintSpace>(inJph->mSpace);
	outJpc->Point1 = to_jpc(inJph->mPoint1);
	outJpc->HingeAxis1 = to_jpc(inJph->mHingeAxis1);
	outJpc->NormalAxis1 = to_jpc(inJph->mNormalAxis1);
	outJpc->Point2 = to_jpc(inJph->mPoint2);
	outJpc->HingeAxis2 = to_jpc(inJph->mHingeAxis2);
	outJpc->NormalAxis2 = to_jpc(inJph->mNormalAxis2);
	outJpc->LimitsMin = inJph->mLimitsMin;
	outJpc->LimitsMax = inJph->mLimitsMax;
	JPC_SpringSettings_to_jpc(&outJpc->LimitsSpringSettings, &inJph->mLimitsSpringSettings);
	outJpc->MaxFrictionTorque = inJph->mMaxFrictionTorque;
	JPC_MotorSettings_to_jpc(&outJpc->MotorSettings, &inJph->mMotorSettings);
}

JPC_IMPL void JPC_HingeConstraintSettings_to_jph(
	const JPC_HingeConstraintSettings* inJpc,
	JPH::HingeConstraintSettings* outJph)
{
	JPC_ConstraintSettings_to_jph(&inJpc->ConstraintSettings, outJph);

	outJph->mSpace = static_cast<JPH::EConstraintSpace>(inJpc->Space);
	outJph->mPoint1 = to_jph(inJpc->Point1);
	outJph->mHingeAxis1 = to_jph(inJpc->HingeAxis1);
	outJph->mNormalAxis1 = to_jph(inJpc->NormalAxis1);
	outJph->mPoint2 = to_jph(inJpc->Point2);
	outJph->mHingeAxis2 = to_jph(inJpc->HingeAxis2);
	outJph->mNormalAxis2 = to_jph(inJpc->NormalAxis2);
	outJph->mLimitsMin = inJpc->LimitsMin;
	outJph->mLimitsMax = inJpc->LimitsMax;
	JPC_SpringSettings_to_jph(&inJpc->LimitsSpringSettings, &outJph->mLimitsSpringSettings);
	outJph->mMaxFrictionTorque = inJpc->MaxFrictionTorque;
	JPC_MotorSettings_to_jph(&inJpc->MotorSettings, &outJph->mMotorSettings);
}

JPC_API void JPC_HingeConstraintSettings_default(JPC_HingeConstraintSettings* settings) {
	JPH::HingeConstraintSettings defaultSettings{};
	JPC_HingeConstraintSettings_to_jpc(settings, &defaultSettings);
}

JPC_API JPC_HingeConstraint* JPC_HingeConstraintSettings_Create(
	const JPC_HingeConstraintSettings* self,
	JPC_Body* inBody1,
	JPC_Body* inBody2)
{
	JPH::HingeConstraintSettings jphSettings;
	JPC_HingeConstraintSettings_to_jph(self, &jphSettings);

	JPH::HingeConstraint* outJph = new JPH::HingeConstraint(*to_jph(inBody1), *to_jph(inBody2), jphSettings);
	return (JPC_HingeConstraint*)outJph;
}

////////////////////////////////////////////////////////////////////////////////
// DistanceConstraintSettings -> TwoBodyConstraintSettings -> ConstraintSettings

JPC_IMPL void JPC_DistanceConstraintSettings_to_jpc(
	JPC_DistanceConstraintSettings* outJpc,
	const JPH::DistanceConstraintSettings* inJph)
{
	JPC_ConstraintSettings_to_jpc(&outJpc->ConstraintSettings, inJph);

	outJpc->Space = static_cast<JPC_ConstraintSpace>(inJph->mSpace);
	outJpc->Point1 = to_jpc(inJph->mPoint1);
	outJpc->Point2 = to_jpc(inJph->mPoint2);
	outJpc->MinDistance = inJph->mMinDistance;
	outJpc->MaxDistance = inJph->mMaxDistance;
	// TODO: Spring settings
}

JPC_IMPL void JPC_DistanceConstraintSettings_to_jph(
	const JPC_DistanceConstraintSettings* inJpc,
	JPH::DistanceConstraintSettings* outJph)
{
	JPC_ConstraintSettings_to_jph(&inJpc->ConstraintSettings, outJph);

	outJph->mSpace = static_cast<JPH::EConstraintSpace>(inJpc->Space);
	outJph->mPoint1 = to_jph(inJpc->Point1);
	outJph->mPoint2 = to_jph(inJpc->Point2);
	outJph->mMinDistance = inJpc->MinDistance;
	outJph->mMaxDistance = inJpc->MaxDistance;
	// TODO: Spring settings
}

JPC_API void JPC_DistanceConstraintSettings_default(JPC_DistanceConstraintSettings* settings) {
	JPH::DistanceConstraintSettings defaultSettings{};
	JPC_DistanceConstraintSettings_to_jpc(settings, &defaultSettings);
}

JPC_API JPC_DistanceConstraint* JPC_DistanceConstraintSettings_Create(
	const JPC_DistanceConstraintSettings* self,
	JPC_Body* inBody1,
	JPC_Body* inBody2)
	{
		JPH::DistanceConstraintSettings jphSettings;
		JPC_DistanceConstraintSettings_to_jph(self, &jphSettings);

		JPH::DistanceConstraint* outJph = new JPH::DistanceConstraint(*to_jph(inBody1), *to_jph(inBody2), jphSettings);
		return (JPC_DistanceConstraint*)outJph;
	}

////////////////////////////////////////////////////////////////////////////////
// SliderConstraintSettings -> TwoBodyConstraintSettings -> ConstraintSettings

JPC_IMPL void JPC_SliderConstraintSettings_to_jpc(
	JPC_SliderConstraintSettings* outJpc,
	const JPH::SliderConstraintSettings* inJph)
{
	JPC_ConstraintSettings_to_jpc(&outJpc->ConstraintSettings, inJph);

	outJpc->Space = static_cast<JPC_ConstraintSpace>(inJph->mSpace);
	outJpc->AutoDetectPoint = inJph->mAutoDetectPoint;
	outJpc->Point1 = to_jpc(inJph->mPoint1);
	outJpc->SliderAxis1 = to_jpc(inJph->mSliderAxis1);
	outJpc->NormalAxis1 = to_jpc(inJph->mNormalAxis1);
	outJpc->Point2 = to_jpc(inJph->mPoint2);
	outJpc->SliderAxis2 = to_jpc(inJph->mSliderAxis2);
	outJpc->NormalAxis2 = to_jpc(inJph->mNormalAxis2);
	outJpc->LimitsMin = inJph->mLimitsMin;
	outJpc->LimitsMax = inJph->mLimitsMax;
	JPC_SpringSettings_to_jpc(&outJpc->LimitsSpringSettings, &inJph->mLimitsSpringSettings);
	outJpc->MaxFrictionForce = inJph->mMaxFrictionForce;
	JPC_MotorSettings_to_jpc(&outJpc->MotorSettings, &inJph->mMotorSettings);
}

JPC_IMPL void JPC_SliderConstraintSettings_to_jph(
	const JPC_SliderConstraintSettings* inJpc,
	JPH::SliderConstraintSettings* outJph)
{
	JPC_ConstraintSettings_to_jph(&inJpc->ConstraintSettings, outJph);

	outJph->mSpace = static_cast<JPH::EConstraintSpace>(inJpc->Space);
	outJph->mAutoDetectPoint = inJpc->AutoDetectPoint;
	outJph->mPoint1 = to_jph(inJpc->Point1);
	outJph->mSliderAxis1 = to_jph(inJpc->SliderAxis1);
	outJph->mNormalAxis1 = to_jph(inJpc->NormalAxis1);
	outJph->mPoint2 = to_jph(inJpc->Point2);
	outJph->mSliderAxis2 = to_jph(inJpc->SliderAxis2);
	outJph->mNormalAxis2 = to_jph(inJpc->NormalAxis2);
	outJph->mLimitsMin = inJpc->LimitsMin;
	outJph->mLimitsMax = inJpc->LimitsMax;
	JPC_SpringSettings_to_jph(&inJpc->LimitsSpringSettings, &outJph->mLimitsSpringSettings);
	outJph->mMaxFrictionForce = inJpc->MaxFrictionForce;
	JPC_MotorSettings_to_jph(&inJpc->MotorSettings, &outJph->mMotorSettings);
}

JPC_API void JPC_SliderConstraintSettings_default(JPC_SliderConstraintSettings* settings) {
	JPH::SliderConstraintSettings defaultSettings{};
	JPC_SliderConstraintSettings_to_jpc(settings, &defaultSettings);
}

JPC_API JPC_SliderConstraint* JPC_SliderConstraintSettings_Create(
	const JPC_SliderConstraintSettings* self,
	JPC_Body* inBody1,
	JPC_Body* inBody2)
{
	JPH::SliderConstraintSettings jphSettings;
	JPC_SliderConstraintSettings_to_jph(self, &jphSettings);

	JPH::SliderConstraint* outJph = new JPH::SliderConstraint(*to_jph(inBody1), *to_jph(inBody2), jphSettings);
	return (JPC_SliderConstraint*)outJph;
}

////////////////////////////////////////////////////////////////////////////////
// Shape -> RefTarget<Shape>

// RefTarget<Shape>
JPC_API uint32_t JPC_Shape_GetRefCount(const JPC_Shape* self) {
	return to_jph(self)->GetRefCount();
}

JPC_API void JPC_Shape_AddRef(const JPC_Shape* self) {
	to_jph(self)->AddRef();
}

JPC_API void JPC_Shape_Release(const JPC_Shape* self) {
	to_jph(self)->Release();
}

// Shape
JPC_API uint64_t JPC_Shape_GetUserData(const JPC_Shape* self) {
	return to_jph(self)->GetUserData();
}

JPC_API void JPC_Shape_SetUserData(JPC_Shape* self, uint64_t userData) {
	to_jph(self)->SetUserData(userData);
}

JPC_API JPC_ShapeType JPC_Shape_GetType(const JPC_Shape* self) {
	return to_jpc(to_jph(self)->GetType());
}

JPC_API JPC_ShapeSubType JPC_Shape_GetSubType(const JPC_Shape* self) {
	return to_jpc(to_jph(self)->GetSubType());
}

JPC_API uint64_t JPC_Shape_GetSubShapeUserData(const JPC_Shape* self, JPC_SubShapeID inSubShapeID) {
	return to_jph(self)->GetSubShapeUserData(JPC_SubShapeID_to_jph(inSubShapeID));
}

JPC_API JPC_Vec3 JPC_Shape_GetCenterOfMass(const JPC_Shape* self) {
	return to_jpc(to_jph(self)->GetCenterOfMass());
}

JPC_API float JPC_Shape_GetVolume(const JPC_Shape* self) {
	return to_jph(self)->GetVolume();
}

////////////////////////////////////////////////////////////////////////////////
// CompoundShape

JPC_API const JPC_Shape* JPC_CompoundShape_GetSubShape_Shape(
	const JPC_CompoundShape* self,
	uint inIdx)
{
	return to_jpc(to_jph(self)->GetSubShape(inIdx).mShape.GetPtr());
}

JPC_API uint32_t JPC_CompoundShape_GetSubShapeIndexFromID(
	const JPC_CompoundShape* self,
	JPC_SubShapeID inSubShapeID,
	JPC_SubShapeID* outRemainder)
{
	JPH::SubShapeID jphRemainder;
	uint32_t res = to_jph(self)->GetSubShapeIndexFromID(JPC_SubShapeID_to_jph(inSubShapeID), jphRemainder);
	*outRemainder = to_jpc(jphRemainder);
	return res;
}

////////////////////////////////////////////////////////////////////////////////
// ShapeSettings

// Unpack a ShapeResult into a bool and two pointers to be friendlier to C.
static bool HandleShapeResult(JPH::ShapeSettings::ShapeResult res, JPC_Shape** outShape, JPC_String** outError) {
	if (res.HasError()) {
		if (outError != nullptr) {
			JPH::String* created = new JPH::String(std::move(res.GetError()));
			*outError = to_jpc(created);
		}

		return false;
	} else {
		JPH::Ref<JPH::Shape> shape = res.Get();
		shape->AddRef();
		*outShape = to_jpc((JPH::Shape*)shape);

		return true;
	}
}

////////////////////////////////////////////////////////////////////////////////
// TriangleShapeSettings

static void to_jph(const JPC_TriangleShapeSettings* input, JPH::TriangleShapeSettings* output) {
	output->mUserData = input->UserData;

	// TODO: Material
	output->mDensity = input->Density;

	output->mV1 = to_jph(input->V1);
	output->mV2 = to_jph(input->V2);
	output->mV3 = to_jph(input->V3);
	output->mConvexRadius = input->ConvexRadius;
}

JPC_API void JPC_TriangleShapeSettings_default(JPC_TriangleShapeSettings* object) {
	object->UserData = 0;

	// TODO: Material
	object->Density = 1000.0;

	object->V1 = {0};
	object->V2 = {0};
	object->V3 = {0};
	object->ConvexRadius = 0.0;
}

JPC_API bool JPC_TriangleShapeSettings_Create(const JPC_TriangleShapeSettings* self, JPC_Shape** outShape, JPC_String** outError) {
	JPH::TriangleShapeSettings settings;
	to_jph(self, &settings);

	return HandleShapeResult(settings.Create(), outShape, outError);
}

////////////////////////////////////////////////////////////////////////////////
// MeshShapeSettings

JPC_IMPL void JPC_MeshShapeSettings_to_jpc_borrowed(
	JPC_MeshShapeSettings* outJpc,
	const JPH::MeshShapeSettings* inJph)
{
	outJpc->UserData = inJph->mUserData;

	outJpc->TriangleVertices = (JPC_Float3*)inJph->mTriangleVertices.data();
	outJpc->TriangleVerticesLen = inJph->mTriangleVertices.size();
	outJpc->IndexedTriangles = (JPC_IndexedTriangle*)inJph->mIndexedTriangles.data();
	outJpc->IndexedTrianglesLen = inJph->mIndexedTriangles.size();
}

JPC_IMPL void JPC_MeshShapeSettings_to_jph(
	const JPC_MeshShapeSettings* inJpc,
	JPH::MeshShapeSettings* outJph)
{
	outJph->mUserData = inJpc->UserData;

	auto triangleVertices = (const JPH::Float3*)inJpc->TriangleVertices;
	outJph->mTriangleVertices = JPH::VertexList(triangleVertices, triangleVertices + inJpc->TriangleVerticesLen);

	auto indexedTriangles = (const JPH::IndexedTriangle*)inJpc->IndexedTriangles;
	outJph->mIndexedTriangles = JPH::IndexedTriangleList(indexedTriangles, indexedTriangles + inJpc->IndexedTrianglesLen);
}

JPC_API void JPC_MeshShapeSettings_default(JPC_MeshShapeSettings* object) {
	JPH::MeshShapeSettings settings;
	JPC_MeshShapeSettings_to_jpc_borrowed(object, &settings);

	// Overwrite all pointers and lengths so that the default value doesn't
	// contain pointers to freed memory.
	object->TriangleVertices = nullptr;
	object->TriangleVerticesLen = 0;
	object->IndexedTriangles = nullptr;
	object->IndexedTrianglesLen = 0;
}

JPC_API bool JPC_MeshShapeSettings_Create(const JPC_MeshShapeSettings* self, JPC_Shape** outShape, JPC_String** outError) {
	JPH::MeshShapeSettings settings;
	JPC_MeshShapeSettings_to_jph(self, &settings);

	// MeshShapeSettings calls Sanitize in its default constructor, but we don't
	// have constructors in C. It's probably fine to always Sanitize.
	settings.Sanitize();

	return HandleShapeResult(settings.Create(), outShape, outError);
}

////////////////////////////////////////////////////////////////////////////////
// BoxShapeSettings

static void to_jph(const JPC_BoxShapeSettings* input, JPH::BoxShapeSettings* output) {
	output->mUserData = input->UserData;

	// TODO: Material
	output->mDensity = input->Density;

	output->mHalfExtent = to_jph(input->HalfExtent);
	output->mConvexRadius = input->ConvexRadius;
}

JPC_API void JPC_BoxShapeSettings_default(JPC_BoxShapeSettings* object) {
	object->UserData = 0;

	// TODO: Material
	object->Density = 1000.0;

	object->HalfExtent = JPC_Vec3{0};
	object->ConvexRadius = 0.0;
}

JPC_API bool JPC_BoxShapeSettings_Create(const JPC_BoxShapeSettings* self, JPC_Shape** outShape, JPC_String** outError) {
	JPH::BoxShapeSettings settings;
	to_jph(self, &settings);

	return HandleShapeResult(settings.Create(), outShape, outError);
}

////////////////////////////////////////////////////////////////////////////////
// SphereShapeSettings

static void to_jph(const JPC_SphereShapeSettings* input, JPH::SphereShapeSettings* output) {
	output->mUserData = input->UserData;

	// TODO: Material
	output->mDensity = input->Density;

	output->mRadius = input->Radius;
}

JPC_API void JPC_SphereShapeSettings_default(JPC_SphereShapeSettings* object) {
	object->UserData = 0;

	// TODO: Material
	object->Density = 1000.0;

	object->Radius = 0.0;
}

JPC_API bool JPC_SphereShapeSettings_Create(const JPC_SphereShapeSettings* self, JPC_Shape** outShape, JPC_String** outError) {
	JPH::SphereShapeSettings settings;
	to_jph(self, &settings);

	return HandleShapeResult(settings.Create(), outShape, outError);
}

////////////////////////////////////////////////////////////////////////////////
// CapsuleShapeSettings

static void to_jph(const JPC_CapsuleShapeSettings* input, JPH::CapsuleShapeSettings* output) {
	output->mUserData = input->UserData;

	// TODO: Material
	output->mDensity = input->Density;

	output->mRadius = input->Radius;
	output->mHalfHeightOfCylinder = input->HalfHeightOfCylinder;
}

JPC_API void JPC_CapsuleShapeSettings_default(JPC_CapsuleShapeSettings* object) {
	object->UserData = 0;

	// TODO: Material
	object->Density = 1000.0;

	object->Radius = 0.0;
	object->HalfHeightOfCylinder = 0.0;
}

JPC_API bool JPC_CapsuleShapeSettings_Create(const JPC_CapsuleShapeSettings* self, JPC_Shape** outShape, JPC_String** outError) {
	JPH::CapsuleShapeSettings settings;
	to_jph(self, &settings);

	return HandleShapeResult(settings.Create(), outShape, outError);
}

////////////////////////////////////////////////////////////////////////////////
// CylinderShapeSettings

static void to_jph(const JPC_CylinderShapeSettings* input, JPH::CylinderShapeSettings* output) {
	output->mUserData = input->UserData;

	// TODO: Material
	output->mDensity = input->Density;

	output->mHalfHeight = input->HalfHeight;
	output->mRadius = input->Radius;
	output->mConvexRadius = input->ConvexRadius;
}

JPC_API void JPC_CylinderShapeSettings_default(JPC_CylinderShapeSettings* object) {
	object->UserData = 0;

	// TODO: Material
	object->Density = 1000.0;

	object->HalfHeight = 0.0;
	object->Radius = 0.0;
	object->ConvexRadius = 0.0;
}

JPC_API bool JPC_CylinderShapeSettings_Create(const JPC_CylinderShapeSettings* self, JPC_Shape** outShape, JPC_String** outError) {
	JPH::CylinderShapeSettings settings;
	to_jph(self, &settings);

	return HandleShapeResult(settings.Create(), outShape, outError);
}

////////////////////////////////////////////////////////////////////////////////
// PlaneShapeSettings

static void to_jph(const JPC_PlaneShapeSettings* input, JPH::PlaneShapeSettings* output) {
	output->mUserData = input->UserData;

	// TODO: Material
	output->mPlane = JPH::Plane(to_jph(input->Normal), input->Constant);
	output->mHalfExtent = input->HalfExtent;
}

JPC_API void JPC_PlaneShapeSettings_default(JPC_PlaneShapeSettings* object) {
	object->UserData = 0;

	// TODO: Material
	object->Normal = JPC_Vec3{0, 1, 0, 1};
	object->Constant = 0.0;
	object->HalfExtent = JPH::PlaneShapeSettings::cDefaultHalfExtent;
}

JPC_API bool JPC_PlaneShapeSettings_Create(const JPC_PlaneShapeSettings* self, JPC_Shape** outShape, JPC_String** outError) {
	JPH::PlaneShapeSettings settings;
	to_jph(self, &settings);

	return HandleShapeResult(settings.Create(), outShape, outError);
}

////////////////////////////////////////////////////////////////////////////////
// ConvexHullShapeSettings

static void to_jph(const JPC_ConvexHullShapeSettings* input, JPH::ConvexHullShapeSettings* output) {
	output->mUserData = input->UserData;

	// TODO: Material
	output->mDensity = input->Density;

	output->mPoints = to_jph(input->Points, input->PointsLen);
	output->mMaxConvexRadius = input->MaxConvexRadius;
	output->mMaxErrorConvexRadius = input->MaxErrorConvexRadius;
	output->mHullTolerance = input->HullTolerance;
}

JPC_API void JPC_ConvexHullShapeSettings_default(JPC_ConvexHullShapeSettings* object) {
	object->UserData = 0;

	// TODO: Material
	object->Density = 1000.0;

	object->Points = nullptr;
	object->PointsLen = 0;
	object->MaxConvexRadius = 0.0;
	object->MaxErrorConvexRadius = 0.05f;
	object->HullTolerance = 1.0e-3f;
}

JPC_API bool JPC_ConvexHullShapeSettings_Create(const JPC_ConvexHullShapeSettings* self, JPC_Shape** outShape, JPC_String** outError) {
	JPH::ConvexHullShapeSettings settings;
	to_jph(self, &settings);

	return HandleShapeResult(settings.Create(), outShape, outError);
}

////////////////////////////////////////////////////////////////////////////////
// CompoundShape::SubShapeSettings

static JPH::CompoundShapeSettings::SubShapeSettings to_jph(const JPC_SubShapeSettings* input) {
	const JPH::Shape* shape = to_jph(input->Shape);

	JPH::CompoundShapeSettings::SubShapeSettings output;
	output.mShape = nullptr;
	output.mShapePtr = shape;
	output.mPosition = to_jph(input->Position);
	output.mRotation = to_jph(input->Rotation);
	output.mUserData = input->UserData;
	return output;
}

static JPH::Array<JPH::CompoundShapeSettings::SubShapeSettings> to_jph(const JPC_SubShapeSettings* src, size_t n) {
	JPH::Array<JPH::CompoundShapeSettings::SubShapeSettings> vec;
	vec.reserve(n);

	for (size_t i = 0; i < n; i++) {
		vec.push_back(to_jph(&src[i]));
	}

	return vec;
}

JPC_API void JPC_SubShapeSettings_default(JPC_SubShapeSettings* object) {
	object->Shape = nullptr;
	object->Position = JPC_Vec3{0};
	object->Rotation = JPC_Quat{0, 0, 0, 1};
	object->UserData = 0;
}

////////////////////////////////////////////////////////////////////////////////
// StaticCompoundShapeSettings -> CompoundShapeSettings -> ShapeSettings

static void to_jph(const JPC_StaticCompoundShapeSettings* input, JPH::StaticCompoundShapeSettings* output) {
	output->mUserData = input->UserData;

	output->mSubShapes = to_jph(input->SubShapes, input->SubShapesLen);
}

JPC_API void JPC_StaticCompoundShapeSettings_default(JPC_StaticCompoundShapeSettings* object) {
	object->UserData = 0;

	object->SubShapes = nullptr;
	object->SubShapesLen = 0;
}

JPC_API bool JPC_StaticCompoundShapeSettings_Create(const JPC_StaticCompoundShapeSettings* self, JPC_Shape** outShape, JPC_String** outError) {
	JPH::StaticCompoundShapeSettings settings;
	to_jph(self, &settings);

	return HandleShapeResult(settings.Create(), outShape, outError);
}

////////////////////////////////////////////////////////////////////////////////
// MutableCompoundShape -> CompoundShape -> Shape

JPC_IMPL JPH::MutableCompoundShape* JPC_MutableCompoundShape_to_jph(JPC_MutableCompoundShape* self) {
	return reinterpret_cast<JPH::MutableCompoundShape*>(self);
}

JPC_API uint JPC_MutableCompoundShape_AddShape(
	JPC_MutableCompoundShape* self,
	JPC_Vec3 inPosition,
	JPC_Quat inRotation,
	const JPC_Shape* inShape,
	uint32_t inUserData)
{
	JPH::MutableCompoundShape* self_jph = JPC_MutableCompoundShape_to_jph(self);

	return self_jph->AddShape(to_jph(inPosition), to_jph(inRotation), to_jph(inShape), inUserData);
}

JPC_API void JPC_MutableCompoundShape_RemoveShape(JPC_MutableCompoundShape* self, uint inIndex) {
	JPH::MutableCompoundShape* self_jph = JPC_MutableCompoundShape_to_jph(self);

	self_jph->RemoveShape(inIndex);
}

JPC_API void JPC_MutableCompoundShape_ModifyShape(JPC_MutableCompoundShape* self, uint inIndex, JPC_Vec3 inPosition, JPC_Quat inRotation) {
	JPH::MutableCompoundShape* self_jph = JPC_MutableCompoundShape_to_jph(self);

	self_jph->ModifyShape(inIndex, to_jph(inPosition), to_jph(inRotation));
}

JPC_API void JPC_MutableCompoundShape_ModifyShape2(JPC_MutableCompoundShape* self, uint inIndex, JPC_Vec3 inPosition, JPC_Quat inRotation, const JPC_Shape* inShape) {
	JPH::MutableCompoundShape* self_jph = JPC_MutableCompoundShape_to_jph(self);

	self_jph->ModifyShape(inIndex, to_jph(inPosition), to_jph(inRotation), to_jph(inShape));
}

JPC_API void JPC_MutableCompoundShape_AdjustCenterOfMass(JPC_MutableCompoundShape* self) {
	JPH::MutableCompoundShape* self_jph = JPC_MutableCompoundShape_to_jph(self);

	self_jph->AdjustCenterOfMass();
}

////////////////////////////////////////////////////////////////////////////////
// MutableCompoundShapeSettings -> CompoundShapeSettings -> ShapeSettings

static void to_jph(const JPC_MutableCompoundShapeSettings* input, JPH::MutableCompoundShapeSettings* output) {
	output->mUserData = input->UserData;

	output->mSubShapes = to_jph(input->SubShapes, input->SubShapesLen);
}

JPC_API void JPC_MutableCompoundShapeSettings_default(JPC_MutableCompoundShapeSettings* object) {
	object->UserData = 0;

	object->SubShapes = nullptr;
	object->SubShapesLen = 0;
}

JPC_API bool JPC_MutableCompoundShapeSettings_Create(const JPC_MutableCompoundShapeSettings* self, JPC_MutableCompoundShape** outShape, JPC_String** outError) {
	JPH::MutableCompoundShapeSettings settings;
	to_jph(self, &settings);

	return HandleShapeResult(settings.Create(), (JPC_Shape**)outShape, outError);
}

////////////////////////////////////////////////////////////////////////////////
// BodyCreationSettings

static JPH::BodyCreationSettings to_jph(const JPC_BodyCreationSettings* settings) {
	JPH::BodyCreationSettings output{};

	output.mPosition = to_jph(settings->Position);
	output.mRotation = to_jph(settings->Rotation);
	output.mLinearVelocity = to_jph(settings->LinearVelocity);
	output.mAngularVelocity = to_jph(settings->AngularVelocity);
	output.mUserData = settings->UserData;
	output.mObjectLayer = settings->ObjectLayer;
	// CollisionGroup
	output.mMotionType = to_jph(settings->MotionType);
	output.mAllowedDOFs = to_jph(settings->AllowedDOFs);
	output.mAllowDynamicOrKinematic = settings->AllowDynamicOrKinematic;
	output.mIsSensor = settings->IsSensor;
	output.mCollideKinematicVsNonDynamic = settings->CollideKinematicVsNonDynamic;
	output.mUseManifoldReduction = settings->UseManifoldReduction;
	output.mApplyGyroscopicForce = settings->ApplyGyroscopicForce;
	output.mMotionQuality = to_jph(settings->MotionQuality);
	output.mEnhancedInternalEdgeRemoval = settings->EnhancedInternalEdgeRemoval;
	output.mAllowSleeping = settings->AllowSleeping;
	output.mFriction = settings->Friction;
	output.mRestitution = settings->Restitution;
	output.mLinearDamping = settings->LinearDamping;
	output.mAngularDamping = settings->AngularDamping;
	output.mMaxLinearVelocity = settings->MaxLinearVelocity;
	output.mMaxAngularVelocity = settings->MaxAngularVelocity;
	output.mGravityFactor = settings->GravityFactor;
	output.mNumVelocityStepsOverride = settings->NumVelocityStepsOverride;
	output.mNumPositionStepsOverride = settings->NumPositionStepsOverride;
	output.mOverrideMassProperties = to_jph(settings->OverrideMassProperties);
	output.mInertiaMultiplier = settings->InertiaMultiplier;
	// output.mMassPropertiesOverride = settings->MassPropertiesOverride;
	output.SetShape(to_jph(settings->Shape));

	return output;
}

JPC_API void JPC_BodyCreationSettings_default(JPC_BodyCreationSettings* settings) {
	JPH::BodyCreationSettings defaultSettings{};

	settings->Position = to_jpc(defaultSettings.mPosition);
	settings->Rotation = to_jpc(defaultSettings.mRotation);
	settings->LinearVelocity = to_jpc(defaultSettings.mLinearVelocity);
	settings->AngularVelocity = to_jpc(defaultSettings.mAngularVelocity);
	settings->UserData = defaultSettings.mUserData;
	settings->ObjectLayer = defaultSettings.mObjectLayer;
	// CollisionGroup
	settings->MotionType = to_jpc(defaultSettings.mMotionType);
	settings->AllowedDOFs = to_jpc(defaultSettings.mAllowedDOFs);
	settings->AllowDynamicOrKinematic = defaultSettings.mAllowDynamicOrKinematic;
	settings->IsSensor = defaultSettings.mIsSensor;
	settings->CollideKinematicVsNonDynamic = defaultSettings.mCollideKinematicVsNonDynamic;
	settings->UseManifoldReduction = defaultSettings.mUseManifoldReduction;
	settings->ApplyGyroscopicForce = defaultSettings.mApplyGyroscopicForce;
	settings->MotionQuality = to_jpc(defaultSettings.mMotionQuality);
	settings->EnhancedInternalEdgeRemoval = defaultSettings.mEnhancedInternalEdgeRemoval;
	settings->AllowSleeping = defaultSettings.mAllowSleeping;
	settings->Friction = defaultSettings.mFriction;
	settings->Restitution = defaultSettings.mRestitution;
	settings->LinearDamping = defaultSettings.mLinearDamping;
	settings->AngularDamping = defaultSettings.mAngularDamping;
	settings->MaxLinearVelocity = defaultSettings.mMaxLinearVelocity;
	settings->MaxAngularVelocity = defaultSettings.mMaxAngularVelocity;
	settings->GravityFactor = defaultSettings.mGravityFactor;
	settings->NumVelocityStepsOverride = defaultSettings.mNumVelocityStepsOverride;
	settings->NumPositionStepsOverride = defaultSettings.mNumPositionStepsOverride;
	settings->OverrideMassProperties = to_jpc(defaultSettings.mOverrideMassProperties);
	settings->InertiaMultiplier = defaultSettings.mInertiaMultiplier;
	// MassPropertiesOverride
}

////////////////////////////////////////////////////////////////////////////////
// Body

JPC_API JPC_BodyID JPC_Body_GetID(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetID());
}

JPC_API JPC_BodyType JPC_Body_GetBodyType(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetBodyType());
}

JPC_API bool JPC_Body_IsRigidBody(const JPC_Body* self) {
	return to_jph(self)->IsRigidBody();
}

JPC_API bool JPC_Body_IsSoftBody(const JPC_Body* self) {
	return to_jph(self)->IsSoftBody();
}

JPC_API bool JPC_Body_IsActive(const JPC_Body* self) {
	return to_jph(self)->IsActive();
}

JPC_API bool JPC_Body_IsStatic(const JPC_Body* self) {
	return to_jph(self)->IsStatic();
}

JPC_API bool JPC_Body_IsKinematic(const JPC_Body* self) {
	return to_jph(self)->IsKinematic();
}

JPC_API bool JPC_Body_IsDynamic(const JPC_Body* self) {
	return to_jph(self)->IsDynamic();
}

JPC_API bool JPC_Body_CanBeKinematicOrDynamic(const JPC_Body* self) {
	return to_jph(self)->CanBeKinematicOrDynamic();
}

JPC_API void JPC_Body_SetIsSensor(JPC_Body* self, bool inIsSensor) {
	to_jph(self)->SetIsSensor(inIsSensor);
}

JPC_API bool JPC_Body_IsSensor(const JPC_Body* self) {
	return to_jph(self)->IsSensor();
}

JPC_API void JPC_Body_SetCollideKinematicVsNonDynamic(JPC_Body* self, bool inCollide) {
	to_jph(self)->SetCollideKinematicVsNonDynamic(inCollide);
}

JPC_API bool JPC_Body_GetCollideKinematicVsNonDynamic(const JPC_Body* self) {
	return to_jph(self)->GetCollideKinematicVsNonDynamic();
}

JPC_API void JPC_Body_SetUseManifoldReduction(JPC_Body* self, bool inUseReduction) {
	to_jph(self)->SetUseManifoldReduction(inUseReduction);
}

JPC_API bool JPC_Body_GetUseManifoldReduction(const JPC_Body* self) {
	return to_jph(self)->GetUseManifoldReduction();
}

JPC_API bool JPC_Body_GetUseManifoldReductionWithBody(const JPC_Body* self, const JPC_Body* inBody2) {
	return to_jph(self)->GetUseManifoldReductionWithBody(*to_jph(inBody2));
}

JPC_API void JPC_Body_SetApplyGyroscopicForce(JPC_Body* self, bool inApply) {
	to_jph(self)->SetApplyGyroscopicForce(inApply);
}

JPC_API bool JPC_Body_GetApplyGyroscopicForce(const JPC_Body* self) {
	return to_jph(self)->GetApplyGyroscopicForce();
}

JPC_API void JPC_Body_SetEnhancedInternalEdgeRemoval(JPC_Body* self, bool inApply) {
	to_jph(self)->SetEnhancedInternalEdgeRemoval(inApply);
}

JPC_API bool JPC_Body_GetEnhancedInternalEdgeRemoval(const JPC_Body* self) {
	return to_jph(self)->GetEnhancedInternalEdgeRemoval();
}

JPC_API bool JPC_Body_GetEnhancedInternalEdgeRemovalWithBody(const JPC_Body* self, const JPC_Body* inBody2) {
	return to_jph(self)->GetEnhancedInternalEdgeRemovalWithBody(*to_jph(inBody2));
}

JPC_API JPC_MotionType JPC_Body_GetMotionType(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetMotionType());
}

JPC_API void JPC_Body_SetMotionType(JPC_Body* self, JPC_MotionType inMotionType) {
	to_jph(self)->SetMotionType(to_jph(inMotionType));
}

JPC_API JPC_BroadPhaseLayer JPC_Body_GetBroadPhaseLayer(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetBroadPhaseLayer());
}

JPC_API JPC_ObjectLayer JPC_Body_GetObjectLayer(const JPC_Body* self) {
	return to_jph(self)->GetObjectLayer();
}

// JPC_API const CollisionGroup & JPC_Body_GetCollisionGroup(const JPC_Body* self);
// JPC_API CollisionGroup & JPC_Body_GetCollisionGroup(JPC_Body* self);
// JPC_API void JPC_Body_SetCollisionGroup(JPC_Body* self, const CollisionGroup &inGroup);

JPC_API bool JPC_Body_GetAllowSleeping(const JPC_Body* self) {
	return to_jph(self)->GetAllowSleeping();
}

JPC_API void JPC_Body_SetAllowSleeping(JPC_Body* self, bool inAllow) {
	to_jph(self)->SetAllowSleeping(inAllow);
}

JPC_API void JPC_Body_ResetSleepTimer(JPC_Body* self) {
	to_jph(self)->ResetSleepTimer();
}

JPC_API float JPC_Body_GetFriction(const JPC_Body* self) {
	return to_jph(self)->GetFriction();
}

JPC_API void JPC_Body_SetFriction(JPC_Body* self, float inFriction) {
	to_jph(self)->SetFriction(inFriction);
}

JPC_API float JPC_Body_GetRestitution(const JPC_Body* self) {
	return to_jph(self)->GetRestitution();
}

JPC_API void JPC_Body_SetRestitution(JPC_Body* self, float inRestitution) {
	to_jph(self)->SetRestitution(inRestitution);
}

JPC_API JPC_Vec3 JPC_Body_GetLinearVelocity(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetLinearVelocity());
}

JPC_API void JPC_Body_SetLinearVelocity(JPC_Body* self, JPC_Vec3 inLinearVelocity) {
	to_jph(self)->SetLinearVelocity(to_jph(inLinearVelocity));
}

JPC_API void JPC_Body_SetLinearVelocityClamped(JPC_Body* self, JPC_Vec3 inLinearVelocity) {
	to_jph(self)->SetLinearVelocityClamped(to_jph(inLinearVelocity));
}

JPC_API JPC_Vec3 JPC_Body_GetAngularVelocity(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetAngularVelocity());
}

JPC_API void JPC_Body_SetAngularVelocity(JPC_Body* self, JPC_Vec3 inAngularVelocity) {
	to_jph(self)->SetAngularVelocity(to_jph(inAngularVelocity));
}

JPC_API void JPC_Body_SetAngularVelocityClamped(JPC_Body* self, JPC_Vec3 inAngularVelocity) {
	to_jph(self)->SetAngularVelocityClamped(to_jph(inAngularVelocity));
}

JPC_API JPC_Vec3 JPC_Body_GetPointVelocityCOM(const JPC_Body* self, JPC_Vec3 inPointRelativeToCOM) {
	return to_jpc(to_jph(self)->GetPointVelocityCOM(to_jph(inPointRelativeToCOM)));
}

JPC_API JPC_Vec3 JPC_Body_GetPointVelocity(const JPC_Body* self, JPC_RVec3 inPoint) {
	return to_jpc(to_jph(self)->GetPointVelocity(to_jph(inPoint)));
}

JPC_API void JPC_Body_AddForce(JPC_Body* self, JPC_Vec3 inForce) {
	to_jph(self)->AddForce(to_jph(inForce));
}

// overload of Body::AddForce
JPC_API void JPC_Body_AddForceAtPoint(JPC_Body* self, JPC_Vec3 inForce, JPC_RVec3 inPosition) {
	to_jph(self)->AddForce(to_jph(inForce), to_jph(inPosition));
}

JPC_API void JPC_Body_AddTorque(JPC_Body* self, JPC_Vec3 inTorque) {
	to_jph(self)->AddTorque(to_jph(inTorque));
}

JPC_API JPC_Vec3 JPC_Body_GetAccumulatedForce(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetAccumulatedForce());
}

JPC_API JPC_Vec3 JPC_Body_GetAccumulatedTorque(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetAccumulatedTorque());
}

JPC_API void JPC_Body_ResetForce(JPC_Body* self) {
	to_jph(self)->ResetForce();
}

JPC_API void JPC_Body_ResetTorque(JPC_Body* self) {
	to_jph(self)->ResetTorque();
}

JPC_API void JPC_Body_ResetMotion(JPC_Body* self) {
	to_jph(self)->ResetMotion();
}

JPC_API void JPC_Body_GetInverseInertia(const JPC_Body* self, JPC_Mat44* outMatrix) {
	to_jph(self)->GetInverseInertia().StoreFloat4x4(reinterpret_cast<JPH::Float4*>(outMatrix));
}

JPC_API void JPC_Body_AddImpulse(JPC_Body* self, JPC_Vec3 inImpulse) {
	to_jph(self)->AddImpulse(to_jph(inImpulse));
}

JPC_API void JPC_Body_AddImpulse2(JPC_Body* self, JPC_Vec3 inImpulse, JPC_RVec3 inPosition) {
	to_jph(self)->AddImpulse(to_jph(inImpulse), to_jph(inPosition));
}

JPC_API void JPC_Body_AddAngularImpulse(JPC_Body* self, JPC_Vec3 inAngularImpulse) {
	to_jph(self)->AddAngularImpulse(to_jph(inAngularImpulse));
}

JPC_API void JPC_Body_MoveKinematic(JPC_Body* self, JPC_RVec3 inTargetPosition, JPC_Quat inTargetRotation, float inDeltaTime) {
	to_jph(self)->MoveKinematic(to_jph(inTargetPosition), to_jph(inTargetRotation), inDeltaTime);
}

JPC_API bool JPC_Body_ApplyBuoyancyImpulse(JPC_Body* self, JPC_RVec3 inSurfacePosition, JPC_Vec3 inSurfaceNormal, float inBuoyancy, float inLinearDrag, float inAngularDrag, JPC_Vec3 inFluidVelocity, JPC_Vec3 inGravity, float inDeltaTime) {
	return to_jph(self)->ApplyBuoyancyImpulse(to_jph(inSurfacePosition), to_jph(inSurfaceNormal), inBuoyancy, inLinearDrag, inAngularDrag, to_jph(inFluidVelocity), to_jph(inGravity), inDeltaTime);
}

JPC_API bool JPC_Body_IsInBroadPhase(const JPC_Body* self) {
	return to_jph(self)->IsInBroadPhase();
}

JPC_API bool JPC_Body_IsCollisionCacheInvalid(const JPC_Body* self) {
	return to_jph(self)->IsCollisionCacheInvalid();
}

JPC_API const JPC_Shape* JPC_Body_GetShape(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetShape());
}

JPC_API JPC_RVec3 JPC_Body_GetPosition(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetPosition());
}

JPC_API JPC_Quat JPC_Body_GetRotation(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetRotation());
}

JPC_API JPC_RMat44 JPC_Body_GetWorldTransform(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetWorldTransform());
}

JPC_API JPC_RVec3 JPC_Body_GetCenterOfMassPosition(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetCenterOfMassPosition());
}

JPC_API JPC_RMat44 JPC_Body_GetCenterOfMassTransform(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetCenterOfMassTransform());
}

JPC_API JPC_RMat44 JPC_Body_GetInverseCenterOfMassTransform(const JPC_Body* self) {
	return to_jpc(to_jph(self)->GetInverseCenterOfMassTransform());
}

JPC_API JPC_AABox JPC_Body_GetWorldSpaceBounds(const JPC_Body* self) {
	const JPH::AABox& box = to_jph(self)->GetWorldSpaceBounds();
	JPC_AABox result;
	result.Min = to_jpc(box.mMin);
	result.Max = to_jpc(box.mMax);
	return result;
}

JPC_API JPC_MotionProperties* JPC_Body_GetMotionProperties(JPC_Body* self) {
	return reinterpret_cast<JPC_MotionProperties*>(to_jph(self)->GetMotionProperties());
}

JPC_API JPC_MotionProperties* JPC_Body_GetMotionPropertiesUnchecked(JPC_Body* self) {
	return reinterpret_cast<JPC_MotionProperties*>(to_jph(self)->GetMotionPropertiesUnchecked());
}

JPC_API uint64_t JPC_Body_GetUserData(const JPC_Body* self) {
	return to_jph(self)->GetUserData();
}

JPC_API void JPC_Body_SetUserData(JPC_Body* self, uint64_t inUserData) {
	to_jph(self)->SetUserData(inUserData);
}

JPC_API JPC_Vec3 JPC_Body_GetWorldSpaceSurfaceNormal(const JPC_Body* self, JPC_SubShapeID inSubShapeID, JPC_RVec3 inPosition) {
	JPH::SubShapeID jph_id = JPC_SubShapeID_to_jph(inSubShapeID);

	return to_jpc(to_jph(self)->GetWorldSpaceSurfaceNormal(jph_id, to_jph(inPosition)));
}

JPC_API JPC_TransformedShape* JPC_Body_GetTransformedShape(const JPC_Body* self) {
	return reinterpret_cast<JPC_TransformedShape*>(new JPH::TransformedShape(to_jph(self)->GetTransformedShape()));
}

////////////////////////////////////////////////////////////////////////////////
// BodyLockRead

JPC_API JPC_BodyLockRead* JPC_BodyLockRead_new(const JPC_BodyLockInterface* interface, JPC_BodyID bodyID) {
	JPH::BodyLockRead* lockRead = new JPH::BodyLockRead(*to_jph(interface), to_jph(bodyID));
	return to_jpc(lockRead);
}

JPC_API void JPC_BodyLockRead_delete(JPC_BodyLockRead* self) {
	delete to_jph(self);
}

JPC_API bool JPC_BodyLockRead_Succeeded(JPC_BodyLockRead* self) {
	return to_jph(self)->Succeeded();
}

JPC_API const JPC_Body* JPC_BodyLockRead_GetBody(JPC_BodyLockRead* self) {
	return to_jpc(&to_jph(self)->GetBody());
}

////////////////////////////////////////////////////////////////////////////////
// BodyLockWrite

JPC_API JPC_BodyLockWrite* JPC_BodyLockWrite_new(const JPC_BodyLockInterface* interface, JPC_BodyID bodyID) {
	JPH::BodyLockWrite* lockWrite = new JPH::BodyLockWrite(*to_jph(interface), to_jph(bodyID));
	return to_jpc(lockWrite);
}

JPC_API void JPC_BodyLockWrite_delete(JPC_BodyLockWrite* self) {
	delete to_jph(self);
}

JPC_API bool JPC_BodyLockWrite_Succeeded(JPC_BodyLockWrite* self) {
	return to_jph(self)->Succeeded();
}

JPC_API JPC_Body* JPC_BodyLockWrite_GetBody(JPC_BodyLockWrite* self) {
	return to_jpc(&to_jph(self)->GetBody());
}

////////////////////////////////////////////////////////////////////////////////
// BodyLockMultiRead

typedef struct JPC_BodyLockMultiRead JPC_BodyLockMultiRead;

JPC_API JPC_BodyLockMultiRead* JPC_BodyLockMultiRead_new(
	const JPC_BodyLockInterface* interface,
	const JPC_BodyID *inBodyIDs,
	int inNumber)
{
	JPH::BodyLockMultiRead* lockRead = new JPH::BodyLockMultiRead(*to_jph(interface), to_jph(inBodyIDs), inNumber);
	return to_jpc(lockRead);
}

JPC_API void JPC_BodyLockMultiRead_delete(JPC_BodyLockMultiRead* self) {
	delete to_jph(self);
}

JPC_API const JPC_Body* JPC_BodyLockMultiRead_GetBody(JPC_BodyLockMultiRead* self, int inBodyIndex) {
	return to_jpc(to_jph(self)->GetBody(inBodyIndex));
}

////////////////////////////////////////////////////////////////////////////////
// BodyLockMultiWrite

typedef struct JPC_BodyLockMultiWrite JPC_BodyLockMultiWrite;

JPC_API JPC_BodyLockMultiWrite* JPC_BodyLockMultiWrite_new(
	const JPC_BodyLockInterface* interface,
	const JPC_BodyID *inBodyIDs,
	int inNumber)
{
	JPH::BodyLockMultiWrite* lockWrite = new JPH::BodyLockMultiWrite(*to_jph(interface), to_jph(inBodyIDs), inNumber);
	return to_jpc(lockWrite);
}

JPC_API void JPC_BodyLockMultiWrite_delete(JPC_BodyLockMultiWrite* self) {
	delete to_jph(self);
}

JPC_API JPC_Body* JPC_BodyLockMultiWrite_GetBody(JPC_BodyLockMultiWrite* self, int inBodyIndex) {
	return to_jpc(to_jph(self)->GetBody(inBodyIndex));
}

////////////////////////////////////////////////////////////////////////////////
// BodyInterface

JPC_API JPC_Body* JPC_BodyInterface_CreateBody(JPC_BodyInterface* self, const JPC_BodyCreationSettings* inSettings) {
	return to_jpc(to_jph(self)->CreateBody(to_jph(inSettings)));
}

// JPC_API JPC_Body* JPC_BodyInterface_CreateSoftBody(JPC_BodyInterface *self, const SoftBodyCreationSettings &inSettings);

JPC_API JPC_Body* JPC_BodyInterface_CreateBodyWithID(JPC_BodyInterface *self, JPC_BodyID inBodyID, const JPC_BodyCreationSettings* inSettings) {
	return to_jpc(to_jph(self)->CreateBodyWithID(to_jph(inBodyID), to_jph(inSettings)));
}

// JPC_API JPC_Body* JPC_BodyInterface_CreateSoftBodyWithID(JPC_BodyInterface *self, JPC_BodyID inBodyID, const SoftBodyCreationSettings* inSettings);

JPC_API JPC_Body* JPC_BodyInterface_CreateBodyWithoutID(const JPC_BodyInterface *self, const JPC_BodyCreationSettings* inSettings) {
	return to_jpc(to_jph(self)->CreateBodyWithoutID(to_jph(inSettings)));
}

// JPC_API JPC_Body* JPC_BodyInterface_CreateSoftBodyWithoutID(const JPC_BodyInterface *self, const SoftBodyCreationSettings* inSettings);

JPC_API void JPC_BodyInterface_DestroyBodyWithoutID(const JPC_BodyInterface *self, JPC_Body *inBody) {
	to_jph(self)->DestroyBodyWithoutID(to_jph(inBody));
}

JPC_API bool JPC_BodyInterface_AssignBodyID(JPC_BodyInterface *self, JPC_Body *ioBody) {
	return to_jph(self)->AssignBodyID(to_jph(ioBody));
}

// JPC_API bool JPC_BodyInterface_AssignBodyID(JPC_BodyInterface *self, JPC_Body *ioBody, JPC_BodyID inBodyID);

JPC_API JPC_Body* JPC_BodyInterface_UnassignBodyID(JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jpc(to_jph(self)->UnassignBodyID(to_jph(inBodyID)));
}

// JPC_API void JPC_BodyInterface_UnassignBodyIDs(JPC_BodyInterface *self, const JPC_BodyID *inBodyIDs, int inNumber, JPC_Body **outBodies) {
// 	return to_jph(self)->UnassignBodyIDs(to_jph(inBodyIDs), inNumber, to_jph(outBodies));
// }

JPC_API void JPC_BodyInterface_DestroyBody(JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	to_jph(self)->DestroyBody(to_jph(inBodyID));
}

// JPC_API void JPC_BodyInterface_DestroyBodies(JPC_BodyInterface *self, const JPC_BodyID *inBodyIDs, int inNumber) {
// 	return to_jph(self)->DestroyBodies(to_jph(inBodyIDs), int inNumber);
// }

JPC_API void JPC_BodyInterface_AddBody(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Activation inActivationMode) {
	to_jph(self)->AddBody(to_jph(inBodyID), to_jph(inActivationMode));
}

JPC_API void JPC_BodyInterface_RemoveBody(JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	to_jph(self)->RemoveBody(to_jph(inBodyID));
}

JPC_API bool JPC_BodyInterface_IsAdded(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jph(self)->IsAdded(to_jph(inBodyID));
}

JPC_API JPC_BodyID JPC_BodyInterface_CreateAndAddBody(JPC_BodyInterface *self, const JPC_BodyCreationSettings* inSettings, JPC_Activation inActivationMode) {
	return to_jpc(to_jph(self)->CreateAndAddBody(to_jph(inSettings), to_jph(inActivationMode)));
}

// JPC_API JPC_BodyID JPC_BodyInterface_CreateAndAddSoftBody(JPC_BodyInterface *self, const SoftBodyCreationSettings &inSettings, JPC_Activation inActivationMode);

JPC_API void* JPC_BodyInterface_AddBodiesPrepare(JPC_BodyInterface *self, JPC_BodyID *ioBodies, int inNumber) {
	return to_jph(self)->AddBodiesPrepare(to_jph(ioBodies), inNumber);
}

JPC_API void JPC_BodyInterface_AddBodiesFinalize(JPC_BodyInterface *self, JPC_BodyID *ioBodies, int inNumber, void* inAddState, JPC_Activation inActivationMode) {
	to_jph(self)->AddBodiesFinalize(to_jph(ioBodies), inNumber, inAddState, to_jph(inActivationMode));
}

JPC_API void JPC_BodyInterface_AddBodiesAbort(JPC_BodyInterface *self, JPC_BodyID *ioBodies, int inNumber, void* inAddState) {
	to_jph(self)->AddBodiesAbort(to_jph(ioBodies), inNumber, inAddState);
}

JPC_API void JPC_BodyInterface_RemoveBodies(JPC_BodyInterface *self, JPC_BodyID *ioBodies, int inNumber) {
	to_jph(self)->RemoveBodies(to_jph(ioBodies), inNumber);
}

JPC_API void JPC_BodyInterface_ActivateBody(JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	to_jph(self)->ActivateBody(to_jph(inBodyID));
}

JPC_API void JPC_BodyInterface_ActivateBodies(JPC_BodyInterface *self, JPC_BodyID *inBodyIDs, int inNumber) {
	to_jph(self)->ActivateBodies(to_jph(inBodyIDs), inNumber);
}

// JPC_API void JPC_BodyInterface_ActivateBodiesInAABox(JPC_BodyInterface *self, const AABox &inBox, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter);

JPC_API void JPC_BodyInterface_DeactivateBody(JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	to_jph(self)->DeactivateBody(to_jph(inBodyID));
}

JPC_API void JPC_BodyInterface_DeactivateBodies(JPC_BodyInterface *self, JPC_BodyID *inBodyIDs, int inNumber) {
	to_jph(self)->DeactivateBodies(to_jph(inBodyIDs), inNumber);
}

JPC_API bool JPC_BodyInterface_IsActive(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jph(self)->IsActive(to_jph(inBodyID));
}

// TwoBodyConstraint * JPC_BodyInterface_CreateConstraint(JPC_BodyInterface *self, const TwoBodyConstraintSettings *inSettings, JPC_BodyID inBodyID1, JPC_BodyID inBodyID2);
// JPC_API void JPC_BodyInterface_ActivateConstraint(JPC_BodyInterface *self, const TwoBodyConstraint *inConstraint);

JPC_API const JPC_Shape* JPC_BodyInterface_GetShape(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	// NOTE: This pointer will only be alive as long as BodyInterface holds onto it!
	return to_jpc(to_jph(self)->GetShape(to_jph(inBodyID)).GetPtr());
}

JPC_API void JPC_BodyInterface_SetShape(const JPC_BodyInterface *self, JPC_BodyID inBodyID, const JPC_Shape *inShape, bool inUpdateMassProperties, JPC_Activation inActivationMode) {
	to_jph(self)->SetShape(to_jph(inBodyID), to_jph(inShape), inUpdateMassProperties, to_jph(inActivationMode));
}

JPC_API void JPC_BodyInterface_NotifyShapeChanged(const JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inPreviousCenterOfMass, bool inUpdateMassProperties, JPC_Activation inActivationMode) {
	to_jph(self)->NotifyShapeChanged(to_jph(inBodyID), to_jph(inPreviousCenterOfMass), inUpdateMassProperties, to_jph(inActivationMode));
}

JPC_API void JPC_BodyInterface_SetObjectLayer(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_ObjectLayer inLayer) {
	to_jph(self)->SetObjectLayer(to_jph(inBodyID), inLayer);
}

JPC_API JPC_ObjectLayer JPC_BodyInterface_GetObjectLayer(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jph(self)->GetObjectLayer(to_jph(inBodyID));
}

JPC_API void JPC_BodyInterface_SetPositionAndRotation(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_RVec3 inPosition, JPC_Quat inRotation, JPC_Activation inActivationMode) {
	to_jph(self)->SetPositionAndRotation(to_jph(inBodyID), to_jph(inPosition), to_jph(inRotation), to_jph(inActivationMode));
}

JPC_API void JPC_BodyInterface_SetPositionAndRotationWhenChanged(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_RVec3 inPosition, JPC_Quat inRotation, JPC_Activation inActivationMode) {
	to_jph(self)->SetPositionAndRotationWhenChanged(to_jph(inBodyID), to_jph(inPosition), to_jph(inRotation), to_jph(inActivationMode));
}

JPC_API void JPC_BodyInterface_GetPositionAndRotation(const JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_RVec3 *outPosition, JPC_Quat *outRotation) {
	JPH::RVec3 outPos{};
	JPH::Quat outRot{};

	to_jph(self)->GetPositionAndRotation(to_jph(inBodyID), outPos, outRot);

	*outPosition = to_jpc(outPos);
	*outRotation = to_jpc(outRot);
}

JPC_API void JPC_BodyInterface_SetPosition(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_RVec3 inPosition, JPC_Activation inActivationMode) {
	to_jph(self)->SetPosition(to_jph(inBodyID), to_jph(inPosition), to_jph(inActivationMode));
}

JPC_API JPC_RVec3 JPC_BodyInterface_GetPosition(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jpc(to_jph(self)->GetPosition(to_jph(inBodyID)));
}

JPC_API JPC_RVec3 JPC_BodyInterface_GetCenterOfMassPosition(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jpc(to_jph(self)->GetCenterOfMassPosition(to_jph(inBodyID)));
}

JPC_API void JPC_BodyInterface_SetRotation(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Quat inRotation, JPC_Activation inActivationMode) {
	to_jph(self)->SetRotation(to_jph(inBodyID), to_jph(inRotation), to_jph(inActivationMode));
}

JPC_API JPC_Quat JPC_BodyInterface_GetRotation(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jpc(to_jph(self)->GetRotation(to_jph(inBodyID)));
}

JPC_API JPC_RMat44 JPC_BodyInterface_GetWorldTransform(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jpc(to_jph(self)->GetWorldTransform(to_jph(inBodyID)));
}

JPC_API JPC_RMat44 JPC_BodyInterface_GetCenterOfMassTransform(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jpc(to_jph(self)->GetCenterOfMassTransform(to_jph(inBodyID)));
}

JPC_API void JPC_BodyInterface_MoveKinematic(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_RVec3 inTargetPosition, JPC_Quat inTargetRotation, float inDeltaTime) {
	to_jph(self)->MoveKinematic(to_jph(inBodyID), to_jph(inTargetPosition), to_jph(inTargetRotation), inDeltaTime);
}

JPC_API void JPC_BodyInterface_SetLinearAndAngularVelocity(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inLinearVelocity, JPC_Vec3 inAngularVelocity) {
	to_jph(self)->SetLinearAndAngularVelocity(to_jph(inBodyID), to_jph(inLinearVelocity), to_jph(inAngularVelocity));
}

JPC_API void JPC_BodyInterface_GetLinearAndAngularVelocity(const JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 *outLinearVelocity, JPC_Vec3 *outAngularVelocity) {
	JPH::Vec3 outLinVel;
	JPH::Vec3 outAngVel;

	to_jph(self)->GetLinearAndAngularVelocity(to_jph(inBodyID), outLinVel, outAngVel);

	*outLinearVelocity = to_jpc(outLinVel);
	*outAngularVelocity = to_jpc(outAngVel);
}

JPC_API void JPC_BodyInterface_SetLinearVelocity(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inLinearVelocity) {
	to_jph(self)->SetLinearVelocity(to_jph(inBodyID), to_jph(inLinearVelocity));
}

JPC_API JPC_Vec3 JPC_BodyInterface_GetLinearVelocity(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jpc(to_jph(self)->GetLinearVelocity(to_jph(inBodyID)));
}

JPC_API void JPC_BodyInterface_AddLinearVelocity(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inLinearVelocity) {
	to_jph(self)->AddLinearVelocity(to_jph(inBodyID), to_jph(inLinearVelocity));
}

JPC_API void JPC_BodyInterface_AddLinearAndAngularVelocity(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inLinearVelocity, JPC_Vec3 inAngularVelocity) {
	to_jph(self)->AddLinearAndAngularVelocity(to_jph(inBodyID), to_jph(inLinearVelocity), to_jph(inAngularVelocity));
}

JPC_API void JPC_BodyInterface_SetAngularVelocity(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inAngularVelocity) {
	to_jph(self)->SetAngularVelocity(to_jph(inBodyID), to_jph(inAngularVelocity));
}

JPC_API JPC_Vec3 JPC_BodyInterface_GetAngularVelocity(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jpc(to_jph(self)->GetAngularVelocity(to_jph(inBodyID)));
}

JPC_API JPC_Vec3 JPC_BodyInterface_GetPointVelocity(const JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_RVec3 inPoint) {
	return to_jpc(to_jph(self)->GetPointVelocity(to_jph(inBodyID), to_jph(inPoint)));
}

JPC_API void JPC_BodyInterface_SetPositionRotationAndVelocity(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_RVec3 inPosition, JPC_Quat inRotation, JPC_Vec3 inLinearVelocity, JPC_Vec3 inAngularVelocity) {
	to_jph(self)->SetPositionRotationAndVelocity(to_jph(inBodyID), to_jph(inPosition), to_jph(inRotation), to_jph(inLinearVelocity), to_jph(inAngularVelocity));
}

JPC_API void JPC_BodyInterface_AddForce(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inForce) {
	to_jph(self)->AddForce(to_jph(inBodyID), to_jph(inForce));
}

// overload of BodyInterface::AddForce
JPC_API void JPC_BodyInterface_AddForceAtPoint(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inForce, JPC_RVec3 inPoint) {
	to_jph(self)->AddForce(to_jph(inBodyID), to_jph(inForce), to_jph(inPoint));
}

JPC_API void JPC_BodyInterface_AddTorque(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inTorque) {
	to_jph(self)->AddTorque(to_jph(inBodyID), to_jph(inTorque));
}

JPC_API void JPC_BodyInterface_AddForceAndTorque(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inForce, JPC_Vec3 inTorque) {
	to_jph(self)->AddForceAndTorque(to_jph(inBodyID), to_jph(inForce), to_jph(inTorque));
}

JPC_API void JPC_BodyInterface_AddImpulse(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inImpulse) {
	to_jph(self)->AddImpulse(to_jph(inBodyID), to_jph(inImpulse));
}

JPC_API void JPC_BodyInterface_AddImpulse3(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inImpulse, JPC_RVec3 inPoint) {
	to_jph(self)->AddImpulse(to_jph(inBodyID), to_jph(inImpulse), to_jph(inPoint));
}

JPC_API void JPC_BodyInterface_AddAngularImpulse(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Vec3 inAngularImpulse) {
	to_jph(self)->AddAngularImpulse(to_jph(inBodyID), to_jph(inAngularImpulse));
}

JPC_API JPC_BodyType JPC_BodyInterface_GetBodyType(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jpc(to_jph(self)->GetBodyType(to_jph(inBodyID)));
}

JPC_API void JPC_BodyInterface_SetMotionType(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_MotionType inMotionType, JPC_Activation inActivationMode) {
	to_jph(self)->SetMotionType(to_jph(inBodyID), to_jph(inMotionType), to_jph(inActivationMode));
}

JPC_API JPC_MotionType JPC_BodyInterface_GetMotionType(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jpc(to_jph(self)->GetMotionType(to_jph(inBodyID)));
}

JPC_API void JPC_BodyInterface_SetMotionQuality(JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_MotionQuality inMotionQuality) {
	to_jph(self)->SetMotionQuality(to_jph(inBodyID), to_jph(inMotionQuality));
}

JPC_API JPC_MotionQuality JPC_BodyInterface_GetMotionQuality(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jpc(to_jph(self)->GetMotionQuality(to_jph(inBodyID)));
}

JPC_API void JPC_BodyInterface_GetInverseInertia(const JPC_BodyInterface *self, JPC_BodyID inBodyID, JPC_Mat44 *outMatrix) {
	to_jph(self)->GetInverseInertia(to_jph(inBodyID)).StoreFloat4x4(reinterpret_cast<JPH::Float4*>(outMatrix));
}

JPC_API void JPC_BodyInterface_SetRestitution(JPC_BodyInterface *self, JPC_BodyID inBodyID, float inRestitution) {
	to_jph(self)->SetRestitution(to_jph(inBodyID), inRestitution);
}

JPC_API float JPC_BodyInterface_GetRestitution(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jph(self)->GetRestitution(to_jph(inBodyID));
}

JPC_API void JPC_BodyInterface_SetFriction(JPC_BodyInterface *self, JPC_BodyID inBodyID, float inFriction) {
	to_jph(self)->SetFriction(to_jph(inBodyID), inFriction);
}

JPC_API float JPC_BodyInterface_GetFriction(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jph(self)->GetFriction(to_jph(inBodyID));
}

JPC_API void JPC_BodyInterface_SetGravityFactor(JPC_BodyInterface *self, JPC_BodyID inBodyID, float inGravityFactor) {
	to_jph(self)->SetGravityFactor(to_jph(inBodyID), inGravityFactor);
}

JPC_API float JPC_BodyInterface_GetGravityFactor(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jph(self)->GetGravityFactor(to_jph(inBodyID));
}

JPC_API void JPC_BodyInterface_SetUseManifoldReduction(JPC_BodyInterface *self, JPC_BodyID inBodyID, bool inUseReduction) {
	to_jph(self)->SetUseManifoldReduction(to_jph(inBodyID), inUseReduction);
}

JPC_API bool JPC_BodyInterface_GetUseManifoldReduction(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jph(self)->GetUseManifoldReduction(to_jph(inBodyID));
}

JPC_API JPC_TransformedShape* JPC_BodyInterface_GetTransformedShape(const JPC_BodyInterface* self, JPC_BodyID inBodyID) {
	return reinterpret_cast<JPC_TransformedShape*>(new JPH::TransformedShape(to_jph(self)->GetTransformedShape(to_jph(inBodyID))));
}

JPC_API uint64_t JPC_BodyInterface_GetUserData(const JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	return to_jph(self)->GetUserData(to_jph(inBodyID));
}

JPC_API void JPC_BodyInterface_SetUserData(const JPC_BodyInterface *self, JPC_BodyID inBodyID, uint64_t inUserData) {
	to_jph(self)->SetUserData(to_jph(inBodyID), inUserData);
}

// const PhysicsMaterial* JPC_BodyInterface_GetMaterial(const JPC_BodyInterface *self, JPC_BodyID inBodyID, const SubShapeID &inSubShapeID);

JPC_API void JPC_BodyInterface_InvalidateContactCache(JPC_BodyInterface *self, JPC_BodyID inBodyID) {
	to_jph(self)->InvalidateContactCache(to_jph(inBodyID));
}

////////////////////////////////////////////////////////////////////////////////
// NarrowPhaseQuery

JPC_API bool JPC_NarrowPhaseQuery_CastRay(const JPC_NarrowPhaseQuery* self, JPC_NarrowPhaseQuery_CastRayArgs* args) {
	JPH::RayCastResult result;

	JPH::RayCastSettings settings;

	JPH::BroadPhaseLayerFilter defaultBplFilter{};
	const JPH::BroadPhaseLayerFilter* bplFilter = &defaultBplFilter;
	if (args->BroadPhaseLayerFilter != nullptr) {
		bplFilter = to_jph(args->BroadPhaseLayerFilter);
	}

	JPH::ObjectLayerFilter defaultOlFilter{};
	const JPH::ObjectLayerFilter* olFilter = &defaultOlFilter;
	if (args->ObjectLayerFilter != nullptr) {
		olFilter = to_jph(args->ObjectLayerFilter);
	}

	JPH::BodyFilter defaultBodyFilter{};
	const JPH::BodyFilter* bodyFilter = &defaultBodyFilter;
	if (args->BodyFilter != nullptr) {
		bodyFilter = to_jph(args->BodyFilter);
	}

	JPH::ShapeFilter defaultShapeFilter{};
	const JPH::ShapeFilter* shapeFilter = &defaultShapeFilter;
	if (args->ShapeFilter != nullptr) {
		shapeFilter = to_jph(args->ShapeFilter);
	}

	JPH::ClosestHitCollisionCollector<JPH::CastRayCollector> collector;

	to_jph(self)->CastRay(
		to_jph(args->Ray),
		settings,
		collector,
		*bplFilter,
		*olFilter,
		*bodyFilter,
		*shapeFilter);

	bool hit = collector.HadHit();
	if (hit) {
		args->Result = to_jpc(collector.mHit);
	}

	return hit;
}

JPC_API void JPC_ShapeCastSettings_default(JPC_ShapeCastSettings* object) {
	JPH::ShapeCastSettings defaultSettings{};
	*object = to_jpc(defaultSettings);
}

JPC_API void JPC_NarrowPhaseQuery_CastShape(const JPC_NarrowPhaseQuery* self, JPC_NarrowPhaseQuery_CastShapeArgs* args) {
	JPH::ShapeCastSettings settings = to_jph(args->Settings);

	JPH::ClosestHitCollisionCollector<JPH::CastShapeCollector> defaultCollector{};
	JPH::CastShapeCollector* collector = &defaultCollector;
	if (args->Collector != nullptr) {
		collector = to_jph(args->Collector);
	}

	JPH::BroadPhaseLayerFilter defaultBplFilter{};
	const JPH::BroadPhaseLayerFilter* bplFilter = &defaultBplFilter;
	if (args->BroadPhaseLayerFilter != nullptr) {
		bplFilter = to_jph(args->BroadPhaseLayerFilter);
	}

	JPH::ObjectLayerFilter defaultOlFilter{};
	const JPH::ObjectLayerFilter* olFilter = &defaultOlFilter;
	if (args->ObjectLayerFilter != nullptr) {
		olFilter = to_jph(args->ObjectLayerFilter);
	}

	JPH::BodyFilter defaultBodyFilter{};
	const JPH::BodyFilter* bodyFilter = &defaultBodyFilter;
	if (args->BodyFilter != nullptr) {
		bodyFilter = to_jph(args->BodyFilter);
	}

	JPH::ShapeFilter defaultShapeFilter{};
	const JPH::ShapeFilter* shapeFilter = &defaultShapeFilter;
	if (args->ShapeFilter != nullptr) {
		shapeFilter = to_jph(args->ShapeFilter);
	}

	to_jph(self)->CastShape(
		to_jph(args->ShapeCast),
		settings,
		to_jph(args->BaseOffset),
		*collector,
		*bplFilter,
		*olFilter,
		*bodyFilter,
		*shapeFilter);
}

JPC_API void JPC_CollideShapeSettings_default(JPC_CollideShapeSettings* object) {
	JPH::CollideShapeSettings defaultSettings{};
	*object = to_jpc(defaultSettings);
}

JPC_API void JPC_NarrowPhaseQuery_CollideShape(const JPC_NarrowPhaseQuery* self, JPC_NarrowPhaseQuery_CollideShapeArgs* args) {
	JPH::CollideShapeSettings settings = to_jph(args->Settings);

	JPH::ClosestHitCollisionCollector<JPH::CollideShapeCollector> defaultCollector{};
	JPH::CollideShapeCollector* collector = &defaultCollector;
	if (args->Collector != nullptr) {
		collector = to_jph(args->Collector);
	}

	JPH::BroadPhaseLayerFilter defaultBplFilter{};
	const JPH::BroadPhaseLayerFilter* bplFilter = &defaultBplFilter;
	if (args->BroadPhaseLayerFilter != nullptr) {
		bplFilter = to_jph(args->BroadPhaseLayerFilter);
	}

	JPH::ObjectLayerFilter defaultOlFilter{};
	const JPH::ObjectLayerFilter* olFilter = &defaultOlFilter;
	if (args->ObjectLayerFilter != nullptr) {
		olFilter = to_jph(args->ObjectLayerFilter);
	}

	JPH::BodyFilter defaultBodyFilter{};
	const JPH::BodyFilter* bodyFilter = &defaultBodyFilter;
	if (args->BodyFilter != nullptr) {
		bodyFilter = to_jph(args->BodyFilter);
	}

	JPH::ShapeFilter defaultShapeFilter{};
	const JPH::ShapeFilter* shapeFilter = &defaultShapeFilter;
	if (args->ShapeFilter != nullptr) {
		shapeFilter = to_jph(args->ShapeFilter);
	}

	to_jph(self)->CollideShape(
		to_jph(args->Shape),
		to_jph(args->ShapeScale),
		to_jph(args->CenterOfMassTransform),
		settings,
		to_jph(args->BaseOffset),
		*collector,
		*bplFilter,
		*olFilter,
		*bodyFilter,
		*shapeFilter);
}

////////////////////////////////////////////////////////////////////////////////
// PhysicsSystem

JPC_API JPC_PhysicsSystem* JPC_PhysicsSystem_new() {
	return to_jpc(new JPH::PhysicsSystem());
}

JPC_API void JPC_PhysicsSystem_Init(
	JPC_PhysicsSystem* self,
	uint inMaxBodies,
	uint inNumBodyMutexes,
	uint inMaxBodyPairs,
	uint inMaxContactConstraints,
	JPC_BroadPhaseLayerInterface* inBroadPhaseLayerInterface,
	JPC_ObjectVsBroadPhaseLayerFilter* inObjectVsBroadPhaseLayerFilter,
	JPC_ObjectLayerPairFilter* inObjectLayerPairFilter)
{
	JPC_BroadPhaseLayerInterfaceBridge* impl_inBroadPhaseLayerInterface = to_jph(inBroadPhaseLayerInterface);
	JPC_ObjectVsBroadPhaseLayerFilterBridge* impl_inObjectVsBroadPhaseLayerFilter = to_jph(inObjectVsBroadPhaseLayerFilter);
	JPC_ObjectLayerPairFilterBridge* impl_inObjectLayerPairFilter = to_jph(inObjectLayerPairFilter);

	to_jph(self)->Init(
		inMaxBodies,
		inNumBodyMutexes,
		inMaxBodyPairs,
		inMaxContactConstraints,
		*impl_inBroadPhaseLayerInterface,
		*impl_inObjectVsBroadPhaseLayerFilter,
		*impl_inObjectLayerPairFilter);
}

JPC_API void JPC_PhysicsSystem_OptimizeBroadPhase(JPC_PhysicsSystem* self) {
	to_jph(self)->OptimizeBroadPhase();
}

JPC_API uint32_t JPC_PhysicsSystem_GetNumBodies(const JPC_PhysicsSystem* self) {
	return to_jph(self)->GetNumBodies();
}

JPC_API uint32_t JPC_PhysicsSystem_GetNumActiveBodies(const JPC_PhysicsSystem* self, JPC_BodyType inType) {
	return to_jph(self)->GetNumActiveBodies(static_cast<JPH::EBodyType>(inType));
}

JPC_API void JPC_PhysicsSystem_GetBodies(const JPC_PhysicsSystem* self, JPC_BodyID* outBodyIDs, uint32_t* outCount) {
	JPH::BodyIDVector ids;
	to_jph(self)->GetBodies(ids);
	uint32_t n = static_cast<uint32_t>(ids.size());
	if (outBodyIDs)
		memcpy(outBodyIDs, ids.data(), n * sizeof(JPC_BodyID));
	*outCount = n;
}

JPC_API void JPC_PhysicsSystem_GetActiveBodies(const JPC_PhysicsSystem* self, JPC_BodyType inType, JPC_BodyID* outBodyIDs, uint32_t* outCount) {
	JPH::BodyIDVector ids;
	to_jph(self)->GetActiveBodies(static_cast<JPH::EBodyType>(inType), ids);
	uint32_t n = static_cast<uint32_t>(ids.size());
	if (outBodyIDs)
		memcpy(outBodyIDs, ids.data(), n * sizeof(JPC_BodyID));
	*outCount = n;
}

JPC_API void JPC_PhysicsSystem_AddConstraint(JPC_PhysicsSystem* self, JPC_Constraint* constraint) {
	to_jph(self)->AddConstraint(to_jph(constraint));
}

JPC_API void JPC_PhysicsSystem_RemoveConstraint(JPC_PhysicsSystem* self, JPC_Constraint* constraint) {
	to_jph(self)->RemoveConstraint(to_jph(constraint));
}

JPC_API void JPC_PhysicsSystem_SetGravity(JPC_PhysicsSystem* self, JPC_Vec3 inGravity) {
	to_jph(self)->SetGravity(to_jph(inGravity));
}

JPC_API JPC_Vec3 JPC_PhysicsSystem_GetGravity(const JPC_PhysicsSystem* self) {
	return to_jpc(to_jph(self)->GetGravity());
}

JPC_API JPC_BodyInterface* JPC_PhysicsSystem_GetBodyInterface(JPC_PhysicsSystem* self) {
	return to_jpc(&to_jph(self)->GetBodyInterface());
}

JPC_API const JPC_BodyLockInterface* JPC_PhysicsSystem_GetBodyLockInterface(JPC_PhysicsSystem* self) {
	return to_jpc(&to_jph(self)->GetBodyLockInterface());
}

JPC_API const JPC_NarrowPhaseQuery* JPC_PhysicsSystem_GetNarrowPhaseQuery(const JPC_PhysicsSystem* self) {
	return to_jpc(&to_jph(self)->GetNarrowPhaseQuery());
}

JPC_API JPC_PhysicsUpdateError JPC_PhysicsSystem_Update(
	JPC_PhysicsSystem* self,
	float inDeltaTime,
	int inCollisionSteps,
	JPC_TempAllocatorImpl *inTempAllocator,
	JPC_JobSystem *inJobSystem)
{
	auto res = to_jph(self)->Update(
		inDeltaTime,
		inCollisionSteps,
		to_jph(inTempAllocator),
		to_jph(inJobSystem));

	return to_integral(res);
}

JPC_API void JPC_PhysicsSystem_DrawBodies(
	JPC_PhysicsSystem* self,
	JPC_BodyManager_DrawSettings* inSettings,
	JPC_DebugRendererSimple* inRenderer,
	[[maybe_unused]] const void* inBodyFilter)
{
	to_jph(self)->DrawBodies(to_jph(*inSettings), to_jph(inRenderer), nullptr);
}

JPC_API void JPC_PhysicsSystem_DrawConstraints(
	JPC_PhysicsSystem* self,
	JPC_DebugRendererSimple* inRenderer)
{
	to_jph(self)->DrawConstraints(to_jph(inRenderer));
}


JPC_API void JPC_PhysicsSystem_SetSimShapeFilter(
	JPC_PhysicsSystem* self,
	const JPC_SimShapeFilter* inShapeFilter)
{
	to_jph(self)->SetSimShapeFilter(to_jph(inShapeFilter));
}

JPC_API void JPC_PhysicsSystem_SetContactListener(
	JPC_PhysicsSystem* self,
	JPC_ContactListener* inContactListener)
{
	to_jph(self)->SetContactListener(to_jph(inContactListener));
}

////////////////////////////////////////////////////////////////////////////////
// Character

OPAQUE_WRAPPER(JPC_Character, JPH::Character)

static JPH::CharacterSettings to_jph_char_settings(const JPC_CharacterSettings* s) {
	JPH::CharacterSettings cs;
	cs.mUp = to_jph(s->Up);
	cs.mSupportingVolume = JPH::Plane(JPH::Vec3(s->SupportingVolume.x, s->SupportingVolume.y, s->SupportingVolume.z), s->SupportingVolume.w);
	cs.mMaxSlopeAngle = s->MaxSlopeAngle;
	cs.mEnhancedInternalEdgeRemoval = s->EnhancedInternalEdgeRemoval;
	cs.mShape = to_jph(s->Shape);
	cs.mLayer = s->Layer;
	cs.mMass = s->Mass;
	cs.mFriction = s->Friction;
	cs.mGravityFactor = s->GravityFactor;
	cs.mAllowedDOFs = to_jph(s->AllowedDOFs);
	return cs;
}

JPC_API void JPC_CharacterSettings_default(JPC_CharacterSettings* settings) {
	JPH::CharacterSettings d;
	settings->Up = to_jpc(d.mUp);
	settings->SupportingVolume = { d.mSupportingVolume.GetNormal().GetX(), d.mSupportingVolume.GetNormal().GetY(), d.mSupportingVolume.GetNormal().GetZ(), d.mSupportingVolume.GetConstant() };
	settings->MaxSlopeAngle = d.mMaxSlopeAngle;
	settings->EnhancedInternalEdgeRemoval = d.mEnhancedInternalEdgeRemoval;
	settings->Shape = nullptr;
	settings->Layer = d.mLayer;
	settings->Mass = d.mMass;
	settings->Friction = d.mFriction;
	settings->GravityFactor = d.mGravityFactor;
	settings->AllowedDOFs = to_jpc(d.mAllowedDOFs);
}

JPC_API JPC_Character* JPC_Character_new(
	const JPC_CharacterSettings* inSettings,
	JPC_RVec3 inPosition,
	JPC_Quat inRotation,
	uint64_t inUserData,
	JPC_PhysicsSystem* inSystem)
{
	JPH::CharacterSettings cs = to_jph_char_settings(inSettings);
	return to_jpc(new JPH::Character(&cs, to_jph(inPosition), to_jph(inRotation), inUserData, to_jph(inSystem)));
}

DESTRUCTOR(JPC_Character)

JPC_API void JPC_Character_AddToPhysicsSystem(JPC_Character* self, JPC_Activation inActivationMode, bool inLockBodies) {
	to_jph(self)->AddToPhysicsSystem(to_jph(inActivationMode), inLockBodies);
}

JPC_API void JPC_Character_RemoveFromPhysicsSystem(JPC_Character* self, bool inLockBodies) {
	to_jph(self)->RemoveFromPhysicsSystem(inLockBodies);
}

JPC_API void JPC_Character_Activate(JPC_Character* self, bool inLockBodies) {
	to_jph(self)->Activate(inLockBodies);
}

JPC_API void JPC_Character_PostSimulation(JPC_Character* self, float inMaxSeparationDistance, bool inLockBodies) {
	to_jph(self)->PostSimulation(inMaxSeparationDistance, inLockBodies);
}

JPC_API JPC_Vec3 JPC_Character_GetLinearVelocity(const JPC_Character* self, bool inLockBodies) {
	return to_jpc(to_jph(self)->GetLinearVelocity(inLockBodies));
}

JPC_API void JPC_Character_SetLinearVelocity(JPC_Character* self, JPC_Vec3 inLinearVelocity, bool inLockBodies) {
	to_jph(self)->SetLinearVelocity(to_jph(inLinearVelocity), inLockBodies);
}

JPC_API void JPC_Character_AddLinearVelocity(JPC_Character* self, JPC_Vec3 inLinearVelocity, bool inLockBodies) {
	to_jph(self)->AddLinearVelocity(to_jph(inLinearVelocity), inLockBodies);
}

JPC_API JPC_BodyID JPC_Character_GetBodyID(const JPC_Character* self) {
	return to_jpc(to_jph(self)->GetBodyID());
}

JPC_API JPC_RVec3 JPC_Character_GetPosition(const JPC_Character* self, bool inLockBodies) {
	return to_jpc(to_jph(self)->GetPosition(inLockBodies));
}

JPC_API void JPC_Character_SetPosition(JPC_Character* self, JPC_RVec3 inPosition, JPC_Activation inActivationMode, bool inLockBodies) {
	to_jph(self)->SetPosition(to_jph(inPosition), to_jph(inActivationMode), inLockBodies);
}

JPC_API JPC_Quat JPC_Character_GetRotation(const JPC_Character* self, bool inLockBodies) {
	return to_jpc(to_jph(self)->GetRotation(inLockBodies));
}

JPC_API void JPC_Character_SetRotation(JPC_Character* self, JPC_Quat inRotation, JPC_Activation inActivationMode, bool inLockBodies) {
	to_jph(self)->SetRotation(to_jph(inRotation), to_jph(inActivationMode), inLockBodies);
}

JPC_API JPC_RVec3 JPC_Character_GetCenterOfMassPosition(const JPC_Character* self, bool inLockBodies) {
	return to_jpc(to_jph(self)->GetCenterOfMassPosition(inLockBodies));
}

JPC_API JPC_GroundState JPC_Character_GetGroundState(const JPC_Character* self) {
	return static_cast<JPC_GroundState>(to_jph(self)->GetGroundState());
}

JPC_API bool JPC_Character_IsSupported(const JPC_Character* self) {
	return to_jph(self)->IsSupported();
}

JPC_API JPC_RVec3 JPC_Character_GetGroundPosition(const JPC_Character* self) {
	return to_jpc(to_jph(self)->GetGroundPosition());
}

JPC_API JPC_Vec3 JPC_Character_GetGroundNormal(const JPC_Character* self) {
	return to_jpc(to_jph(self)->GetGroundNormal());
}

JPC_API JPC_Vec3 JPC_Character_GetGroundVelocity(const JPC_Character* self) {
	return to_jpc(to_jph(self)->GetGroundVelocity());
}

JPC_API JPC_BodyID JPC_Character_GetGroundBodyID(const JPC_Character* self) {
	return to_jpc(to_jph(self)->GetGroundBodyID());
}

JPC_API uint64_t JPC_Character_GetGroundUserData(const JPC_Character* self) {
	return to_jph(self)->GetGroundUserData();
}

JPC_API JPC_ObjectLayer JPC_Character_GetLayer(const JPC_Character* self) {
	return to_jph(self)->GetLayer();
}

JPC_API void JPC_Character_SetLayer(JPC_Character* self, JPC_ObjectLayer inLayer, bool inLockBodies) {
	to_jph(self)->SetLayer(inLayer, inLockBodies);
}

JPC_API void JPC_Character_SetUp(JPC_Character* self, JPC_Vec3 inUp) {
	to_jph(self)->SetUp(to_jph(inUp));
}

JPC_API JPC_Vec3 JPC_Character_GetUp(const JPC_Character* self) {
	return to_jpc(to_jph(self)->GetUp());
}

JPC_API bool JPC_Character_SetShape(JPC_Character* self, const JPC_Shape* inShape, float inMaxPenetrationDepth, bool inLockBodies) {
	return to_jph(self)->SetShape(to_jph(inShape), inMaxPenetrationDepth, inLockBodies);
}

////////////////////////////////////////////////////////////////////////////////
// CharacterVirtual

OPAQUE_WRAPPER(JPC_CharacterVirtual, JPH::CharacterVirtual)

static JPH::CharacterVirtualSettings to_jph_char_virtual_settings(const JPC_CharacterVirtualSettings* s) {
	JPH::CharacterVirtualSettings cs;
	cs.mUp = to_jph(s->Up);
	cs.mSupportingVolume = JPH::Plane(JPH::Vec3(s->SupportingVolume.x, s->SupportingVolume.y, s->SupportingVolume.z), s->SupportingVolume.w);
	cs.mMaxSlopeAngle = s->MaxSlopeAngle;
	cs.mEnhancedInternalEdgeRemoval = s->EnhancedInternalEdgeRemoval;
	cs.mShape = to_jph(s->Shape);
	cs.mID = JPH::CharacterID(s->ID);
	cs.mMass = s->Mass;
	cs.mMaxStrength = s->MaxStrength;
	cs.mShapeOffset = to_jph(s->ShapeOffset);
	cs.mBackFaceMode = static_cast<JPH::EBackFaceMode>(s->BackFaceMode);
	cs.mPredictiveContactDistance = s->PredictiveContactDistance;
	cs.mMaxCollisionIterations = s->MaxCollisionIterations;
	cs.mMaxConstraintIterations = s->MaxConstraintIterations;
	cs.mMinTimeRemaining = s->MinTimeRemaining;
	cs.mCollisionTolerance = s->CollisionTolerance;
	cs.mCharacterPadding = s->CharacterPadding;
	cs.mMaxNumHits = s->MaxNumHits;
	cs.mHitReductionCosMaxAngle = s->HitReductionCosMaxAngle;
	cs.mPenetrationRecoverySpeed = s->PenetrationRecoverySpeed;
	cs.mInnerBodyShape = to_jph(s->InnerBodyShape);
	cs.mInnerBodyIDOverride = to_jph(s->InnerBodyIDOverride);
	cs.mInnerBodyLayer = s->InnerBodyLayer;
	return cs;
}

JPC_API void JPC_CharacterVirtualSettings_default(JPC_CharacterVirtualSettings* settings) {
	JPH::CharacterVirtualSettings d;
	settings->Up = to_jpc(d.mUp);
	settings->SupportingVolume = { d.mSupportingVolume.GetNormal().GetX(), d.mSupportingVolume.GetNormal().GetY(), d.mSupportingVolume.GetNormal().GetZ(), d.mSupportingVolume.GetConstant() };
	settings->MaxSlopeAngle = d.mMaxSlopeAngle;
	settings->EnhancedInternalEdgeRemoval = d.mEnhancedInternalEdgeRemoval;
	settings->Shape = nullptr;
	settings->ID = d.mID.GetValue();
	settings->Mass = d.mMass;
	settings->MaxStrength = d.mMaxStrength;
	settings->ShapeOffset = to_jpc(d.mShapeOffset);
	settings->BackFaceMode = static_cast<JPC_BackFaceMode>(d.mBackFaceMode);
	settings->PredictiveContactDistance = d.mPredictiveContactDistance;
	settings->MaxCollisionIterations = d.mMaxCollisionIterations;
	settings->MaxConstraintIterations = d.mMaxConstraintIterations;
	settings->MinTimeRemaining = d.mMinTimeRemaining;
	settings->CollisionTolerance = d.mCollisionTolerance;
	settings->CharacterPadding = d.mCharacterPadding;
	settings->MaxNumHits = d.mMaxNumHits;
	settings->HitReductionCosMaxAngle = d.mHitReductionCosMaxAngle;
	settings->PenetrationRecoverySpeed = d.mPenetrationRecoverySpeed;
	settings->InnerBodyShape = nullptr;
	settings->InnerBodyIDOverride = to_jpc(d.mInnerBodyIDOverride);
	settings->InnerBodyLayer = d.mInnerBodyLayer;
}

JPC_API JPC_CharacterVirtual* JPC_CharacterVirtual_new(
	const JPC_CharacterVirtualSettings* inSettings,
	JPC_RVec3 inPosition,
	JPC_Quat inRotation,
	uint64_t inUserData,
	JPC_PhysicsSystem* inSystem)
{
	JPH::CharacterVirtualSettings cs = to_jph_char_virtual_settings(inSettings);
	return to_jpc(new JPH::CharacterVirtual(&cs, to_jph(inPosition), to_jph(inRotation), inUserData, to_jph(inSystem)));
}

DESTRUCTOR(JPC_CharacterVirtual)

JPC_API JPC_CharacterID JPC_CharacterVirtual_GetID(const JPC_CharacterVirtual* self) {
	return to_jph(self)->GetID().GetValue();
}

JPC_API JPC_Vec3 JPC_CharacterVirtual_GetLinearVelocity(const JPC_CharacterVirtual* self) {
	return to_jpc(to_jph(self)->GetLinearVelocity());
}

JPC_API void JPC_CharacterVirtual_SetLinearVelocity(JPC_CharacterVirtual* self, JPC_Vec3 inLinearVelocity) {
	to_jph(self)->SetLinearVelocity(to_jph(inLinearVelocity));
}

JPC_API JPC_RVec3 JPC_CharacterVirtual_GetPosition(const JPC_CharacterVirtual* self) {
	return to_jpc(to_jph(self)->GetPosition());
}

JPC_API void JPC_CharacterVirtual_SetPosition(JPC_CharacterVirtual* self, JPC_RVec3 inPosition) {
	to_jph(self)->SetPosition(to_jph(inPosition));
}

JPC_API JPC_Quat JPC_CharacterVirtual_GetRotation(const JPC_CharacterVirtual* self) {
	return to_jpc(to_jph(self)->GetRotation());
}

JPC_API void JPC_CharacterVirtual_SetRotation(JPC_CharacterVirtual* self, JPC_Quat inRotation) {
	to_jph(self)->SetRotation(to_jph(inRotation));
}

JPC_API JPC_RVec3 JPC_CharacterVirtual_GetCenterOfMassPosition(const JPC_CharacterVirtual* self) {
	return to_jpc(to_jph(self)->GetCenterOfMassPosition());
}

JPC_API float JPC_CharacterVirtual_GetMass(const JPC_CharacterVirtual* self) {
	return to_jph(self)->GetMass();
}

JPC_API void JPC_CharacterVirtual_SetMass(JPC_CharacterVirtual* self, float inMass) {
	to_jph(self)->SetMass(inMass);
}

JPC_API float JPC_CharacterVirtual_GetMaxStrength(const JPC_CharacterVirtual* self) {
	return to_jph(self)->GetMaxStrength();
}

JPC_API void JPC_CharacterVirtual_SetMaxStrength(JPC_CharacterVirtual* self, float inMaxStrength) {
	to_jph(self)->SetMaxStrength(inMaxStrength);
}

JPC_API float JPC_CharacterVirtual_GetPenetrationRecoverySpeed(const JPC_CharacterVirtual* self) {
	return to_jph(self)->GetPenetrationRecoverySpeed();
}

JPC_API void JPC_CharacterVirtual_SetPenetrationRecoverySpeed(JPC_CharacterVirtual* self, float inSpeed) {
	to_jph(self)->SetPenetrationRecoverySpeed(inSpeed);
}

JPC_API bool JPC_CharacterVirtual_GetEnhancedInternalEdgeRemoval(const JPC_CharacterVirtual* self) {
	return to_jph(self)->GetEnhancedInternalEdgeRemoval();
}

JPC_API void JPC_CharacterVirtual_SetEnhancedInternalEdgeRemoval(JPC_CharacterVirtual* self, bool inApply) {
	to_jph(self)->SetEnhancedInternalEdgeRemoval(inApply);
}

JPC_API float JPC_CharacterVirtual_GetCharacterPadding(const JPC_CharacterVirtual* self) {
	return to_jph(self)->GetCharacterPadding();
}

JPC_API uint JPC_CharacterVirtual_GetMaxNumHits(const JPC_CharacterVirtual* self) {
	return to_jph(self)->GetMaxNumHits();
}

JPC_API void JPC_CharacterVirtual_SetMaxNumHits(JPC_CharacterVirtual* self, uint inMaxHits) {
	to_jph(self)->SetMaxNumHits(inMaxHits);
}

JPC_API float JPC_CharacterVirtual_GetHitReductionCosMaxAngle(const JPC_CharacterVirtual* self) {
	return to_jph(self)->GetHitReductionCosMaxAngle();
}

JPC_API void JPC_CharacterVirtual_SetHitReductionCosMaxAngle(JPC_CharacterVirtual* self, float inCosMaxAngle) {
	to_jph(self)->SetHitReductionCosMaxAngle(inCosMaxAngle);
}

JPC_API bool JPC_CharacterVirtual_GetMaxHitsExceeded(const JPC_CharacterVirtual* self) {
	return to_jph(self)->GetMaxHitsExceeded();
}

JPC_API uint64_t JPC_CharacterVirtual_GetUserData(const JPC_CharacterVirtual* self) {
	return to_jph(self)->GetUserData();
}

JPC_API void JPC_CharacterVirtual_SetUserData(JPC_CharacterVirtual* self, uint64_t inUserData) {
	to_jph(self)->SetUserData(inUserData);
}

JPC_API JPC_BodyID JPC_CharacterVirtual_GetInnerBodyID(const JPC_CharacterVirtual* self) {
	return to_jpc(to_jph(self)->GetInnerBodyID());
}

JPC_API void JPC_CharacterVirtual_SetUp(JPC_CharacterVirtual* self, JPC_Vec3 inUp) {
	to_jph(self)->SetUp(to_jph(inUp));
}

JPC_API JPC_Vec3 JPC_CharacterVirtual_GetUp(const JPC_CharacterVirtual* self) {
	return to_jpc(to_jph(self)->GetUp());
}

JPC_API JPC_GroundState JPC_CharacterVirtual_GetGroundState(const JPC_CharacterVirtual* self) {
	return static_cast<JPC_GroundState>(to_jph(self)->GetGroundState());
}

JPC_API bool JPC_CharacterVirtual_IsSupported(const JPC_CharacterVirtual* self) {
	return to_jph(self)->IsSupported();
}

JPC_API JPC_RVec3 JPC_CharacterVirtual_GetGroundPosition(const JPC_CharacterVirtual* self) {
	return to_jpc(to_jph(self)->GetGroundPosition());
}

JPC_API JPC_Vec3 JPC_CharacterVirtual_GetGroundNormal(const JPC_CharacterVirtual* self) {
	return to_jpc(to_jph(self)->GetGroundNormal());
}

JPC_API JPC_Vec3 JPC_CharacterVirtual_GetGroundVelocity(const JPC_CharacterVirtual* self) {
	return to_jpc(to_jph(self)->GetGroundVelocity());
}

JPC_API JPC_BodyID JPC_CharacterVirtual_GetGroundBodyID(const JPC_CharacterVirtual* self) {
	return to_jpc(to_jph(self)->GetGroundBodyID());
}

JPC_API uint64_t JPC_CharacterVirtual_GetGroundUserData(const JPC_CharacterVirtual* self) {
	return to_jph(self)->GetGroundUserData();
}

JPC_API JPC_Vec3 JPC_CharacterVirtual_CancelVelocityTowardsSteepSlopes(const JPC_CharacterVirtual* self, JPC_Vec3 inDesiredVelocity) {
	return to_jpc(to_jph(self)->CancelVelocityTowardsSteepSlopes(to_jph(inDesiredVelocity)));
}

JPC_API void JPC_ExtendedUpdateSettings_default(JPC_ExtendedUpdateSettings* settings) {
	JPH::CharacterVirtual::ExtendedUpdateSettings d;
	settings->StickToFloorStepDown = to_jpc(d.mStickToFloorStepDown);
	settings->WalkStairsStepUp = to_jpc(d.mWalkStairsStepUp);
	settings->WalkStairsMinStepForward = d.mWalkStairsMinStepForward;
	settings->WalkStairsStepForwardTest = d.mWalkStairsStepForwardTest;
	settings->WalkStairsCosAngleForwardContact = d.mWalkStairsCosAngleForwardContact;
	settings->WalkStairsStepDownExtra = to_jpc(d.mWalkStairsStepDownExtra);
}

static JPH::CharacterVirtual::ExtendedUpdateSettings to_jph_extended_settings(const JPC_ExtendedUpdateSettings* s) {
	JPH::CharacterVirtual::ExtendedUpdateSettings out;
	out.mStickToFloorStepDown = to_jph(s->StickToFloorStepDown);
	out.mWalkStairsStepUp = to_jph(s->WalkStairsStepUp);
	out.mWalkStairsMinStepForward = s->WalkStairsMinStepForward;
	out.mWalkStairsStepForwardTest = s->WalkStairsStepForwardTest;
	out.mWalkStairsCosAngleForwardContact = s->WalkStairsCosAngleForwardContact;
	out.mWalkStairsStepDownExtra = to_jph(s->WalkStairsStepDownExtra);
	return out;
}

JPC_API void JPC_CharacterVirtual_Update(JPC_CharacterVirtual* self, JPC_CharacterVirtual_UpdateArgs* args) {
	to_jph(self)->Update(
		args->DeltaTime,
		to_jph(args->Gravity),
		*to_jph(args->BroadPhaseLayerFilter),
		*to_jph(args->ObjectLayerFilter),
		*to_jph(args->BodyFilter),
		*to_jph(args->ShapeFilter),
		*to_jph(args->TempAllocator));
}

JPC_API bool JPC_CharacterVirtual_CanWalkStairs(const JPC_CharacterVirtual* self, JPC_Vec3 inLinearVelocity) {
	return to_jph(self)->CanWalkStairs(to_jph(inLinearVelocity));
}

JPC_API bool JPC_CharacterVirtual_WalkStairs(JPC_CharacterVirtual* self, JPC_CharacterVirtual_WalkStairsArgs* args) {
	return to_jph(self)->WalkStairs(
		args->DeltaTime,
		to_jph(args->StepUp),
		to_jph(args->StepForward),
		to_jph(args->StepForwardTest),
		to_jph(args->StepDownExtra),
		*to_jph(args->BroadPhaseLayerFilter),
		*to_jph(args->ObjectLayerFilter),
		*to_jph(args->BodyFilter),
		*to_jph(args->ShapeFilter),
		*to_jph(args->TempAllocator));
}

JPC_API bool JPC_CharacterVirtual_StickToFloor(JPC_CharacterVirtual* self, JPC_CharacterVirtual_StickToFloorArgs* args) {
	return to_jph(self)->StickToFloor(
		to_jph(args->StepDown),
		*to_jph(args->BroadPhaseLayerFilter),
		*to_jph(args->ObjectLayerFilter),
		*to_jph(args->BodyFilter),
		*to_jph(args->ShapeFilter),
		*to_jph(args->TempAllocator));
}

JPC_API void JPC_CharacterVirtual_ExtendedUpdate(JPC_CharacterVirtual* self, JPC_CharacterVirtual_ExtendedUpdateArgs* args) {
	auto settings = to_jph_extended_settings(&args->Settings);
	to_jph(self)->ExtendedUpdate(
		args->DeltaTime,
		to_jph(args->Gravity),
		settings,
		*to_jph(args->BroadPhaseLayerFilter),
		*to_jph(args->ObjectLayerFilter),
		*to_jph(args->BodyFilter),
		*to_jph(args->ShapeFilter),
		*to_jph(args->TempAllocator));
}

JPC_API void JPC_CharacterVirtual_RefreshContacts(JPC_CharacterVirtual* self, JPC_CharacterVirtual_RefreshContactsArgs* args) {
	to_jph(self)->RefreshContacts(
		*to_jph(args->BroadPhaseLayerFilter),
		*to_jph(args->ObjectLayerFilter),
		*to_jph(args->BodyFilter),
		*to_jph(args->ShapeFilter),
		*to_jph(args->TempAllocator));
}

JPC_API void JPC_CharacterVirtual_UpdateGroundVelocity(JPC_CharacterVirtual* self) {
	to_jph(self)->UpdateGroundVelocity();
}

JPC_API bool JPC_CharacterVirtual_SetShape(JPC_CharacterVirtual* self, JPC_CharacterVirtual_SetShapeArgs* args) {
	return to_jph(self)->SetShape(
		to_jph(args->Shape),
		args->MaxPenetrationDepth,
		*to_jph(args->BroadPhaseLayerFilter),
		*to_jph(args->ObjectLayerFilter),
		*to_jph(args->BodyFilter),
		*to_jph(args->ShapeFilter),
		*to_jph(args->TempAllocator));
}

////////////////////////////////////////////////////////////////////////////////
// HeightFieldShapeSettings

JPC_API void JPC_HeightFieldShapeSettings_default(JPC_HeightFieldShapeSettings* object) {
	JPH::HeightFieldShapeSettings s;

	object->UserData = s.mUserData;
	object->Offset = to_jpc(s.mOffset);
	object->Scale = to_jpc(s.mScale);
	object->SampleCount = s.mSampleCount;
	object->MinHeightValue = s.mMinHeightValue;
	object->MaxHeightValue = s.mMaxHeightValue;
	object->BlockSize = s.mBlockSize;
	object->BitsPerSample = s.mBitsPerSample;
	object->HeightSamples = nullptr;
	object->HeightSamplesLen = 0;
	object->MaterialIndices = nullptr;
	object->MaterialIndicesLen = 0;
	object->ActiveEdgeCosThresholdAngle = s.mActiveEdgeCosThresholdAngle;
}

JPC_API bool JPC_HeightFieldShapeSettings_Create(const JPC_HeightFieldShapeSettings* self, JPC_Shape** outShape, JPC_String** outError) {
	JPH::HeightFieldShapeSettings s;

	s.mUserData = self->UserData;
	s.mOffset = to_jph(self->Offset);
	s.mScale = to_jph(self->Scale);
	s.mSampleCount = self->SampleCount;
	s.mMinHeightValue = self->MinHeightValue;
	s.mMaxHeightValue = self->MaxHeightValue;
	s.mBlockSize = self->BlockSize;
	s.mBitsPerSample = self->BitsPerSample;
	s.mHeightSamples.assign(self->HeightSamples, self->HeightSamples + self->HeightSamplesLen);
	s.mMaterialIndices.assign(self->MaterialIndices, self->MaterialIndices + self->MaterialIndicesLen);
	s.mActiveEdgeCosThresholdAngle = self->ActiveEdgeCosThresholdAngle;

	return HandleShapeResult(s.Create(), outShape, outError);
}

////////////////////////////////////////////////////////////////////////////////
// ScaledShapeSettings

JPC_API void JPC_ScaledShapeSettings_default(JPC_ScaledShapeSettings* object) {
	object->UserData = 0;
	object->InnerShape = nullptr;
	object->Scale = to_jpc(JPH::Vec3::sReplicate(1.0f));
}

JPC_API bool JPC_ScaledShapeSettings_Create(const JPC_ScaledShapeSettings* self, JPC_Shape** outShape, JPC_String** outError) {
	JPH::ScaledShapeSettings s(to_jph(self->InnerShape), to_jph(self->Scale));
	s.mUserData = self->UserData;

	return HandleShapeResult(s.Create(), outShape, outError);
}

////////////////////////////////////////////////////////////////////////////////
// RotatedTranslatedShapeSettings

JPC_API void JPC_RotatedTranslatedShapeSettings_default(JPC_RotatedTranslatedShapeSettings* object) {
	object->UserData = 0;
	object->InnerShape = nullptr;
	object->Position = to_jpc(JPH::Vec3::sZero());
	object->Rotation = to_jpc(JPH::Quat::sIdentity());
}

JPC_API bool JPC_RotatedTranslatedShapeSettings_Create(const JPC_RotatedTranslatedShapeSettings* self, JPC_Shape** outShape, JPC_String** outError) {
	JPH::RotatedTranslatedShapeSettings s(to_jph(self->Position), to_jph(self->Rotation), to_jph(self->InnerShape));
	s.mUserData = self->UserData;

	return HandleShapeResult(s.Create(), outShape, outError);
}

////////////////////////////////////////////////////////////////////////////////
// OffsetCenterOfMassShapeSettings

JPC_API void JPC_OffsetCenterOfMassShapeSettings_default(JPC_OffsetCenterOfMassShapeSettings* object) {
	object->UserData = 0;
	object->InnerShape = nullptr;
	object->Offset = to_jpc(JPH::Vec3::sZero());
}

JPC_API bool JPC_OffsetCenterOfMassShapeSettings_Create(const JPC_OffsetCenterOfMassShapeSettings* self, JPC_Shape** outShape, JPC_String** outError) {
	JPH::OffsetCenterOfMassShapeSettings s(to_jph(self->Offset), to_jph(self->InnerShape));
	s.mUserData = self->UserData;

	return HandleShapeResult(s.Create(), outShape, outError);
}

////////////////////////////////////////////////////////////////////////////////
// TaperedCapsuleShapeSettings

JPC_API void JPC_TaperedCapsuleShapeSettings_default(JPC_TaperedCapsuleShapeSettings* object) {
	JPH::TaperedCapsuleShapeSettings s;

	object->UserData = s.mUserData;
	object->Density = s.mDensity;
	object->HalfHeightOfTaperedCylinder = s.mHalfHeightOfTaperedCylinder;
	object->TopRadius = s.mTopRadius;
	object->BottomRadius = s.mBottomRadius;
}

JPC_API bool JPC_TaperedCapsuleShapeSettings_Create(const JPC_TaperedCapsuleShapeSettings* self, JPC_Shape** outShape, JPC_String** outError) {
	JPH::TaperedCapsuleShapeSettings s;

	s.mUserData = self->UserData;
	s.mDensity = self->Density;
	s.mHalfHeightOfTaperedCylinder = self->HalfHeightOfTaperedCylinder;
	s.mTopRadius = self->TopRadius;
	s.mBottomRadius = self->BottomRadius;

	return HandleShapeResult(s.Create(), outShape, outError);
}

////////////////////////////////////////////////////////////////////////////////
// TaperedCylinderShapeSettings

JPC_API void JPC_TaperedCylinderShapeSettings_default(JPC_TaperedCylinderShapeSettings* object) {
	JPH::TaperedCylinderShapeSettings s;

	object->UserData = s.mUserData;
	object->Density = s.mDensity;
	object->HalfHeight = s.mHalfHeight;
	object->TopRadius = s.mTopRadius;
	object->BottomRadius = s.mBottomRadius;
	object->ConvexRadius = s.mConvexRadius;
}

JPC_API bool JPC_TaperedCylinderShapeSettings_Create(const JPC_TaperedCylinderShapeSettings* self, JPC_Shape** outShape, JPC_String** outError) {
	JPH::TaperedCylinderShapeSettings s;

	s.mUserData = self->UserData;
	s.mDensity = self->Density;
	s.mHalfHeight = self->HalfHeight;
	s.mTopRadius = self->TopRadius;
	s.mBottomRadius = self->BottomRadius;
	s.mConvexRadius = self->ConvexRadius;

	return HandleShapeResult(s.Create(), outShape, outError);
}

////////////////////////////////////////////////////////////////////////////////
// EmptyShapeSettings

JPC_API void JPC_EmptyShapeSettings_default(JPC_EmptyShapeSettings* object) {
	object->UserData = 0;
	object->CenterOfMass = to_jpc(JPH::Vec3::sZero());
}

JPC_API bool JPC_EmptyShapeSettings_Create(const JPC_EmptyShapeSettings* self, JPC_Shape** outShape, JPC_String** outError) {
	JPH::EmptyShapeSettings s(to_jph(self->CenterOfMass));
	s.mUserData = self->UserData;

	return HandleShapeResult(s.Create(), outShape, outError);
}

////////////////////////////////////////////////////////////////////////////////
// Body collision group

JPC_API JPC_CollisionGroup JPC_Body_GetCollisionGroup(const JPC_Body* self) {
	return JPC_CollisionGroup_to_jpc(&to_jph(self)->GetCollisionGroup());
}

JPC_API void JPC_Body_SetCollisionGroup(JPC_Body* self, const JPC_CollisionGroup* inGroup) {
	to_jph(self)->SetCollisionGroup(JPC_CollisionGroup_to_jph(inGroup));
}

////////////////////////////////////////////////////////////////////////////////
// BodyInterface extras

JPC_API void JPC_BodyInterface_ActivateConstraint(JPC_BodyInterface* self, const JPC_TwoBodyConstraint* inConstraint) {
	to_jph(self)->ActivateConstraint(to_jph(inConstraint));
}

JPC_API void JPC_BodyInterface_ActivateBodiesInAABox(
	JPC_BodyInterface* self,
	const JPC_AABox* inBox,
	const JPC_BroadPhaseLayerFilter* inBroadPhaseLayerFilter,
	const JPC_ObjectLayerFilter* inObjectLayerFilter)
{
	JPH::AABox box(to_jph(inBox->Min), to_jph(inBox->Max));
	to_jph(self)->ActivateBodiesInAABox(box, *to_jph(inBroadPhaseLayerFilter), *to_jph(inObjectLayerFilter));
}

////////////////////////////////////////////////////////////////////////////////
// PointConstraint -> TwoBodyConstraint -> Constraint -> RefTarget<Constraint>

OPAQUE_WRAPPER(JPC_PointConstraint, JPH::PointConstraint);

JPC_API JPC_Vec3 JPC_PointConstraint_GetTotalLambdaPosition(const JPC_PointConstraint* self) {
	return to_jpc(to_jph(self)->GetTotalLambdaPosition());
}

JPC_IMPL void JPC_PointConstraintSettings_to_jpc(
	JPC_PointConstraintSettings* outJpc,
	const JPH::PointConstraintSettings* inJph)
{
	JPC_ConstraintSettings_to_jpc(&outJpc->ConstraintSettings, inJph);

	outJpc->Space = static_cast<JPC_ConstraintSpace>(inJph->mSpace);
	outJpc->Point1 = to_jpc(inJph->mPoint1);
	outJpc->Point2 = to_jpc(inJph->mPoint2);
}

JPC_IMPL void JPC_PointConstraintSettings_to_jph(
	const JPC_PointConstraintSettings* inJpc,
	JPH::PointConstraintSettings* outJph)
{
	JPC_ConstraintSettings_to_jph(&inJpc->ConstraintSettings, outJph);

	outJph->mSpace = static_cast<JPH::EConstraintSpace>(inJpc->Space);
	outJph->mPoint1 = to_jph(inJpc->Point1);
	outJph->mPoint2 = to_jph(inJpc->Point2);
}

JPC_API void JPC_PointConstraintSettings_default(JPC_PointConstraintSettings* settings) {
	JPH::PointConstraintSettings defaultSettings{};
	JPC_PointConstraintSettings_to_jpc(settings, &defaultSettings);
}

JPC_API JPC_PointConstraint* JPC_PointConstraintSettings_Create(
	const JPC_PointConstraintSettings* self,
	JPC_Body* inBody1,
	JPC_Body* inBody2)
{
	JPH::PointConstraintSettings jphSettings;
	JPC_PointConstraintSettings_to_jph(self, &jphSettings);

	JPH::PointConstraint* outJph = new JPH::PointConstraint(*to_jph(inBody1), *to_jph(inBody2), jphSettings);
	return (JPC_PointConstraint*)outJph;
}

////////////////////////////////////////////////////////////////////////////////
// ConeConstraint -> TwoBodyConstraint -> Constraint -> RefTarget<Constraint>

OPAQUE_WRAPPER(JPC_ConeConstraint, JPH::ConeConstraint);

JPC_API void JPC_ConeConstraint_SetHalfConeAngle(JPC_ConeConstraint* self, float inHalfConeAngle) {
	to_jph(self)->SetHalfConeAngle(inHalfConeAngle);
}

JPC_API float JPC_ConeConstraint_GetCosHalfConeAngle(const JPC_ConeConstraint* self) {
	return to_jph(self)->GetCosHalfConeAngle();
}

JPC_API JPC_Vec3 JPC_ConeConstraint_GetTotalLambdaPosition(const JPC_ConeConstraint* self) {
	return to_jpc(to_jph(self)->GetTotalLambdaPosition());
}

JPC_API float JPC_ConeConstraint_GetTotalLambdaRotation(const JPC_ConeConstraint* self) {
	return to_jph(self)->GetTotalLambdaRotation();
}

JPC_IMPL void JPC_ConeConstraintSettings_to_jpc(
	JPC_ConeConstraintSettings* outJpc,
	const JPH::ConeConstraintSettings* inJph)
{
	JPC_ConstraintSettings_to_jpc(&outJpc->ConstraintSettings, inJph);

	outJpc->Space = static_cast<JPC_ConstraintSpace>(inJph->mSpace);
	outJpc->Point1 = to_jpc(inJph->mPoint1);
	outJpc->TwistAxis1 = to_jpc(inJph->mTwistAxis1);
	outJpc->Point2 = to_jpc(inJph->mPoint2);
	outJpc->TwistAxis2 = to_jpc(inJph->mTwistAxis2);
	outJpc->HalfConeAngle = inJph->mHalfConeAngle;
}

JPC_IMPL void JPC_ConeConstraintSettings_to_jph(
	const JPC_ConeConstraintSettings* inJpc,
	JPH::ConeConstraintSettings* outJph)
{
	JPC_ConstraintSettings_to_jph(&inJpc->ConstraintSettings, outJph);

	outJph->mSpace = static_cast<JPH::EConstraintSpace>(inJpc->Space);
	outJph->mPoint1 = to_jph(inJpc->Point1);
	outJph->mTwistAxis1 = to_jph(inJpc->TwistAxis1);
	outJph->mPoint2 = to_jph(inJpc->Point2);
	outJph->mTwistAxis2 = to_jph(inJpc->TwistAxis2);
	outJph->mHalfConeAngle = inJpc->HalfConeAngle;
}

JPC_API void JPC_ConeConstraintSettings_default(JPC_ConeConstraintSettings* settings) {
	JPH::ConeConstraintSettings defaultSettings{};
	JPC_ConeConstraintSettings_to_jpc(settings, &defaultSettings);
}

JPC_API JPC_ConeConstraint* JPC_ConeConstraintSettings_Create(
	const JPC_ConeConstraintSettings* self,
	JPC_Body* inBody1,
	JPC_Body* inBody2)
{
	JPH::ConeConstraintSettings jphSettings;
	JPC_ConeConstraintSettings_to_jph(self, &jphSettings);

	JPH::ConeConstraint* outJph = new JPH::ConeConstraint(*to_jph(inBody1), *to_jph(inBody2), jphSettings);
	return (JPC_ConeConstraint*)outJph;
}

////////////////////////////////////////////////////////////////////////////////
// PulleyConstraint -> TwoBodyConstraint -> Constraint -> RefTarget<Constraint>

OPAQUE_WRAPPER(JPC_PulleyConstraint, JPH::PulleyConstraint);

JPC_API void JPC_PulleyConstraint_SetLength(JPC_PulleyConstraint* self, float inMinLength, float inMaxLength) {
	to_jph(self)->SetLength(inMinLength, inMaxLength);
}

JPC_API float JPC_PulleyConstraint_GetMinLength(const JPC_PulleyConstraint* self) {
	return to_jph(self)->GetMinLength();
}

JPC_API float JPC_PulleyConstraint_GetMaxLength(const JPC_PulleyConstraint* self) {
	return to_jph(self)->GetMaxLength();
}

JPC_API float JPC_PulleyConstraint_GetCurrentLength(const JPC_PulleyConstraint* self) {
	return to_jph(self)->GetCurrentLength();
}

JPC_API float JPC_PulleyConstraint_GetTotalLambdaPosition(const JPC_PulleyConstraint* self) {
	return to_jph(self)->GetTotalLambdaPosition();
}

JPC_IMPL void JPC_PulleyConstraintSettings_to_jpc(
	JPC_PulleyConstraintSettings* outJpc,
	const JPH::PulleyConstraintSettings* inJph)
{
	JPC_ConstraintSettings_to_jpc(&outJpc->ConstraintSettings, inJph);

	outJpc->Space = static_cast<JPC_ConstraintSpace>(inJph->mSpace);
	outJpc->BodyPoint1 = to_jpc(inJph->mBodyPoint1);
	outJpc->FixedPoint1 = to_jpc(inJph->mFixedPoint1);
	outJpc->BodyPoint2 = to_jpc(inJph->mBodyPoint2);
	outJpc->FixedPoint2 = to_jpc(inJph->mFixedPoint2);
	outJpc->Ratio = inJph->mRatio;
	outJpc->MinLength = inJph->mMinLength;
	outJpc->MaxLength = inJph->mMaxLength;
}

JPC_IMPL void JPC_PulleyConstraintSettings_to_jph(
	const JPC_PulleyConstraintSettings* inJpc,
	JPH::PulleyConstraintSettings* outJph)
{
	JPC_ConstraintSettings_to_jph(&inJpc->ConstraintSettings, outJph);

	outJph->mSpace = static_cast<JPH::EConstraintSpace>(inJpc->Space);
	outJph->mBodyPoint1 = to_jph(inJpc->BodyPoint1);
	outJph->mFixedPoint1 = to_jph(inJpc->FixedPoint1);
	outJph->mBodyPoint2 = to_jph(inJpc->BodyPoint2);
	outJph->mFixedPoint2 = to_jph(inJpc->FixedPoint2);
	outJph->mRatio = inJpc->Ratio;
	outJph->mMinLength = inJpc->MinLength;
	outJph->mMaxLength = inJpc->MaxLength;
}

JPC_API void JPC_PulleyConstraintSettings_default(JPC_PulleyConstraintSettings* settings) {
	JPH::PulleyConstraintSettings defaultSettings{};
	JPC_PulleyConstraintSettings_to_jpc(settings, &defaultSettings);
}

JPC_API JPC_PulleyConstraint* JPC_PulleyConstraintSettings_Create(
	const JPC_PulleyConstraintSettings* self,
	JPC_Body* inBody1,
	JPC_Body* inBody2)
{
	JPH::PulleyConstraintSettings jphSettings;
	JPC_PulleyConstraintSettings_to_jph(self, &jphSettings);

	JPH::PulleyConstraint* outJph = new JPH::PulleyConstraint(*to_jph(inBody1), *to_jph(inBody2), jphSettings);
	return (JPC_PulleyConstraint*)outJph;
}

////////////////////////////////////////////////////////////////////////////////
// GearConstraint -> TwoBodyConstraint -> Constraint -> RefTarget<Constraint>

OPAQUE_WRAPPER(JPC_GearConstraint, JPH::GearConstraint);

JPC_API void JPC_GearConstraint_SetConstraints(
	JPC_GearConstraint* self,
	const JPC_Constraint* inGear1,
	const JPC_Constraint* inGear2)
{
	to_jph(self)->SetConstraints(to_jph(inGear1), to_jph(inGear2));
}

JPC_API float JPC_GearConstraint_GetTotalLambda(const JPC_GearConstraint* self) {
	return to_jph(self)->GetTotalLambda();
}

JPC_IMPL void JPC_GearConstraintSettings_to_jpc(
	JPC_GearConstraintSettings* outJpc,
	const JPH::GearConstraintSettings* inJph)
{
	JPC_ConstraintSettings_to_jpc(&outJpc->ConstraintSettings, inJph);

	outJpc->Space = static_cast<JPC_ConstraintSpace>(inJph->mSpace);
	outJpc->HingeAxis1 = to_jpc(inJph->mHingeAxis1);
	outJpc->HingeAxis2 = to_jpc(inJph->mHingeAxis2);
	outJpc->Ratio = inJph->mRatio;
}

JPC_IMPL void JPC_GearConstraintSettings_to_jph(
	const JPC_GearConstraintSettings* inJpc,
	JPH::GearConstraintSettings* outJph)
{
	JPC_ConstraintSettings_to_jph(&inJpc->ConstraintSettings, outJph);

	outJph->mSpace = static_cast<JPH::EConstraintSpace>(inJpc->Space);
	outJph->mHingeAxis1 = to_jph(inJpc->HingeAxis1);
	outJph->mHingeAxis2 = to_jph(inJpc->HingeAxis2);
	outJph->mRatio = inJpc->Ratio;
}

JPC_API void JPC_GearConstraintSettings_default(JPC_GearConstraintSettings* settings) {
	JPH::GearConstraintSettings defaultSettings{};
	JPC_GearConstraintSettings_to_jpc(settings, &defaultSettings);
}

JPC_API JPC_GearConstraint* JPC_GearConstraintSettings_Create(
	const JPC_GearConstraintSettings* self,
	JPC_Body* inBody1,
	JPC_Body* inBody2)
{
	JPH::GearConstraintSettings jphSettings;
	JPC_GearConstraintSettings_to_jph(self, &jphSettings);

	JPH::GearConstraint* outJph = new JPH::GearConstraint(*to_jph(inBody1), *to_jph(inBody2), jphSettings);
	return (JPC_GearConstraint*)outJph;
}

////////////////////////////////////////////////////////////////////////////////
// RackAndPinionConstraint -> TwoBodyConstraint -> Constraint -> RefTarget<Constraint>

OPAQUE_WRAPPER(JPC_RackAndPinionConstraint, JPH::RackAndPinionConstraint);

JPC_API void JPC_RackAndPinionConstraint_SetConstraints(
	JPC_RackAndPinionConstraint* self,
	const JPC_Constraint* inPinion,
	const JPC_Constraint* inRack)
{
	to_jph(self)->SetConstraints(to_jph(inPinion), to_jph(inRack));
}

JPC_API float JPC_RackAndPinionConstraint_GetTotalLambda(const JPC_RackAndPinionConstraint* self) {
	return to_jph(self)->GetTotalLambda();
}

JPC_IMPL void JPC_RackAndPinionConstraintSettings_to_jpc(
	JPC_RackAndPinionConstraintSettings* outJpc,
	const JPH::RackAndPinionConstraintSettings* inJph)
{
	JPC_ConstraintSettings_to_jpc(&outJpc->ConstraintSettings, inJph);

	outJpc->Space = static_cast<JPC_ConstraintSpace>(inJph->mSpace);
	outJpc->HingeAxis = to_jpc(inJph->mHingeAxis);
	outJpc->SliderAxis = to_jpc(inJph->mSliderAxis);
	outJpc->Ratio = inJph->mRatio;
}

JPC_IMPL void JPC_RackAndPinionConstraintSettings_to_jph(
	const JPC_RackAndPinionConstraintSettings* inJpc,
	JPH::RackAndPinionConstraintSettings* outJph)
{
	JPC_ConstraintSettings_to_jph(&inJpc->ConstraintSettings, outJph);

	outJph->mSpace = static_cast<JPH::EConstraintSpace>(inJpc->Space);
	outJph->mHingeAxis = to_jph(inJpc->HingeAxis);
	outJph->mSliderAxis = to_jph(inJpc->SliderAxis);
	outJph->mRatio = inJpc->Ratio;
}

JPC_API void JPC_RackAndPinionConstraintSettings_default(JPC_RackAndPinionConstraintSettings* settings) {
	JPH::RackAndPinionConstraintSettings defaultSettings{};
	JPC_RackAndPinionConstraintSettings_to_jpc(settings, &defaultSettings);
}

JPC_API JPC_RackAndPinionConstraint* JPC_RackAndPinionConstraintSettings_Create(
	const JPC_RackAndPinionConstraintSettings* self,
	JPC_Body* inBody1,
	JPC_Body* inBody2)
{
	JPH::RackAndPinionConstraintSettings jphSettings;
	JPC_RackAndPinionConstraintSettings_to_jph(self, &jphSettings);

	JPH::RackAndPinionConstraint* outJph = new JPH::RackAndPinionConstraint(*to_jph(inBody1), *to_jph(inBody2), jphSettings);
	return (JPC_RackAndPinionConstraint*)outJph;
}

////////////////////////////////////////////////////////////////////////////////
// SwingTwistConstraint -> TwoBodyConstraint -> Constraint -> RefTarget<Constraint>

OPAQUE_WRAPPER(JPC_SwingTwistConstraint, JPH::SwingTwistConstraint);

JPC_API float JPC_SwingTwistConstraint_GetNormalHalfConeAngle(const JPC_SwingTwistConstraint* self) {
	return to_jph(self)->GetNormalHalfConeAngle();
}

JPC_API void JPC_SwingTwistConstraint_SetNormalHalfConeAngle(JPC_SwingTwistConstraint* self, float inAngle) {
	to_jph(self)->SetNormalHalfConeAngle(inAngle);
}

JPC_API float JPC_SwingTwistConstraint_GetPlaneHalfConeAngle(const JPC_SwingTwistConstraint* self) {
	return to_jph(self)->GetPlaneHalfConeAngle();
}

JPC_API void JPC_SwingTwistConstraint_SetPlaneHalfConeAngle(JPC_SwingTwistConstraint* self, float inAngle) {
	to_jph(self)->SetPlaneHalfConeAngle(inAngle);
}

JPC_API float JPC_SwingTwistConstraint_GetTwistMinAngle(const JPC_SwingTwistConstraint* self) {
	return to_jph(self)->GetTwistMinAngle();
}

JPC_API void JPC_SwingTwistConstraint_SetTwistMinAngle(JPC_SwingTwistConstraint* self, float inAngle) {
	to_jph(self)->SetTwistMinAngle(inAngle);
}

JPC_API float JPC_SwingTwistConstraint_GetTwistMaxAngle(const JPC_SwingTwistConstraint* self) {
	return to_jph(self)->GetTwistMaxAngle();
}

JPC_API void JPC_SwingTwistConstraint_SetTwistMaxAngle(JPC_SwingTwistConstraint* self, float inAngle) {
	to_jph(self)->SetTwistMaxAngle(inAngle);
}

JPC_API float JPC_SwingTwistConstraint_GetMaxFrictionTorque(const JPC_SwingTwistConstraint* self) {
	return to_jph(self)->GetMaxFrictionTorque();
}

JPC_API void JPC_SwingTwistConstraint_SetMaxFrictionTorque(JPC_SwingTwistConstraint* self, float inFrictionTorque) {
	to_jph(self)->SetMaxFrictionTorque(inFrictionTorque);
}

JPC_API JPC_MotorState JPC_SwingTwistConstraint_GetSwingMotorState(const JPC_SwingTwistConstraint* self) {
	return to_jpc(to_jph(self)->GetSwingMotorState());
}

JPC_API void JPC_SwingTwistConstraint_SetSwingMotorState(JPC_SwingTwistConstraint* self, JPC_MotorState inState) {
	to_jph(self)->SetSwingMotorState(to_jph(inState));
}

JPC_API JPC_MotorState JPC_SwingTwistConstraint_GetTwistMotorState(const JPC_SwingTwistConstraint* self) {
	return to_jpc(to_jph(self)->GetTwistMotorState());
}

JPC_API void JPC_SwingTwistConstraint_SetTwistMotorState(JPC_SwingTwistConstraint* self, JPC_MotorState inState) {
	to_jph(self)->SetTwistMotorState(to_jph(inState));
}

JPC_API JPC_Vec3 JPC_SwingTwistConstraint_GetTargetAngularVelocityCS(const JPC_SwingTwistConstraint* self) {
	return to_jpc(to_jph(self)->GetTargetAngularVelocityCS());
}

JPC_API void JPC_SwingTwistConstraint_SetTargetAngularVelocityCS(JPC_SwingTwistConstraint* self, JPC_Vec3 inAngularVelocity) {
	to_jph(self)->SetTargetAngularVelocityCS(to_jph(inAngularVelocity));
}

JPC_API JPC_Quat JPC_SwingTwistConstraint_GetTargetOrientationCS(const JPC_SwingTwistConstraint* self) {
	return to_jpc(to_jph(self)->GetTargetOrientationCS());
}

JPC_API void JPC_SwingTwistConstraint_SetTargetOrientationCS(JPC_SwingTwistConstraint* self, JPC_Quat inOrientation) {
	to_jph(self)->SetTargetOrientationCS(to_jph(inOrientation));
}

JPC_API JPC_Quat JPC_SwingTwistConstraint_GetRotationInConstraintSpace(const JPC_SwingTwistConstraint* self) {
	return to_jpc(to_jph(self)->GetRotationInConstraintSpace());
}

JPC_API JPC_Vec3 JPC_SwingTwistConstraint_GetTotalLambdaPosition(const JPC_SwingTwistConstraint* self) {
	return to_jpc(to_jph(self)->GetTotalLambdaPosition());
}

JPC_API float JPC_SwingTwistConstraint_GetTotalLambdaTwist(const JPC_SwingTwistConstraint* self) {
	return to_jph(self)->GetTotalLambdaTwist();
}

JPC_API float JPC_SwingTwistConstraint_GetTotalLambdaSwingY(const JPC_SwingTwistConstraint* self) {
	return to_jph(self)->GetTotalLambdaSwingY();
}

JPC_API float JPC_SwingTwistConstraint_GetTotalLambdaSwingZ(const JPC_SwingTwistConstraint* self) {
	return to_jph(self)->GetTotalLambdaSwingZ();
}

JPC_API JPC_Vec3 JPC_SwingTwistConstraint_GetTotalLambdaMotor(const JPC_SwingTwistConstraint* self) {
	return to_jpc(to_jph(self)->GetTotalLambdaMotor());
}

JPC_IMPL void JPC_SwingTwistConstraintSettings_to_jpc(
	JPC_SwingTwistConstraintSettings* outJpc,
	const JPH::SwingTwistConstraintSettings* inJph)
{
	JPC_ConstraintSettings_to_jpc(&outJpc->ConstraintSettings, inJph);

	outJpc->Space = static_cast<JPC_ConstraintSpace>(inJph->mSpace);
	outJpc->Position1 = to_jpc(inJph->mPosition1);
	outJpc->TwistAxis1 = to_jpc(inJph->mTwistAxis1);
	outJpc->PlaneAxis1 = to_jpc(inJph->mPlaneAxis1);
	outJpc->Position2 = to_jpc(inJph->mPosition2);
	outJpc->TwistAxis2 = to_jpc(inJph->mTwistAxis2);
	outJpc->PlaneAxis2 = to_jpc(inJph->mPlaneAxis2);
	outJpc->SwingType = static_cast<JPC_SwingType>(inJph->mSwingType);
	outJpc->NormalHalfConeAngle = inJph->mNormalHalfConeAngle;
	outJpc->PlaneHalfConeAngle = inJph->mPlaneHalfConeAngle;
	outJpc->TwistMinAngle = inJph->mTwistMinAngle;
	outJpc->TwistMaxAngle = inJph->mTwistMaxAngle;
	outJpc->MaxFrictionTorque = inJph->mMaxFrictionTorque;
	JPC_MotorSettings_to_jpc(&outJpc->SwingMotorSettings, &inJph->mSwingMotorSettings);
	JPC_MotorSettings_to_jpc(&outJpc->TwistMotorSettings, &inJph->mTwistMotorSettings);
}

JPC_IMPL void JPC_SwingTwistConstraintSettings_to_jph(
	const JPC_SwingTwistConstraintSettings* inJpc,
	JPH::SwingTwistConstraintSettings* outJph)
{
	JPC_ConstraintSettings_to_jph(&inJpc->ConstraintSettings, outJph);

	outJph->mSpace = static_cast<JPH::EConstraintSpace>(inJpc->Space);
	outJph->mPosition1 = to_jph(inJpc->Position1);
	outJph->mTwistAxis1 = to_jph(inJpc->TwistAxis1);
	outJph->mPlaneAxis1 = to_jph(inJpc->PlaneAxis1);
	outJph->mPosition2 = to_jph(inJpc->Position2);
	outJph->mTwistAxis2 = to_jph(inJpc->TwistAxis2);
	outJph->mPlaneAxis2 = to_jph(inJpc->PlaneAxis2);
	outJph->mSwingType = static_cast<JPH::ESwingType>(inJpc->SwingType);
	outJph->mNormalHalfConeAngle = inJpc->NormalHalfConeAngle;
	outJph->mPlaneHalfConeAngle = inJpc->PlaneHalfConeAngle;
	outJph->mTwistMinAngle = inJpc->TwistMinAngle;
	outJph->mTwistMaxAngle = inJpc->TwistMaxAngle;
	outJph->mMaxFrictionTorque = inJpc->MaxFrictionTorque;
	JPC_MotorSettings_to_jph(&inJpc->SwingMotorSettings, &outJph->mSwingMotorSettings);
	JPC_MotorSettings_to_jph(&inJpc->TwistMotorSettings, &outJph->mTwistMotorSettings);
}

JPC_API void JPC_SwingTwistConstraintSettings_default(JPC_SwingTwistConstraintSettings* settings) {
	JPH::SwingTwistConstraintSettings defaultSettings{};
	JPC_SwingTwistConstraintSettings_to_jpc(settings, &defaultSettings);
}

JPC_API JPC_SwingTwistConstraint* JPC_SwingTwistConstraintSettings_Create(
	const JPC_SwingTwistConstraintSettings* self,
	JPC_Body* inBody1,
	JPC_Body* inBody2)
{
	JPH::SwingTwistConstraintSettings jphSettings;
	JPC_SwingTwistConstraintSettings_to_jph(self, &jphSettings);

	JPH::SwingTwistConstraint* outJph = new JPH::SwingTwistConstraint(*to_jph(inBody1), *to_jph(inBody2), jphSettings);
	return (JPC_SwingTwistConstraint*)outJph;
}

////////////////////////////////////////////////////////////////////////////////
// PathConstraintPath -> RefTarget<PathConstraintPath>

OPAQUE_WRAPPER(JPC_PathConstraintPath, JPH::PathConstraintPath)

JPC_API uint32_t JPC_PathConstraintPath_GetRefCount(const JPC_PathConstraintPath* self) {
	return to_jph(self)->GetRefCount();
}

JPC_API void JPC_PathConstraintPath_AddRef(const JPC_PathConstraintPath* self) {
	to_jph(self)->AddRef();
}

JPC_API void JPC_PathConstraintPath_Release(const JPC_PathConstraintPath* self) {
	to_jph(self)->Release();
}

////////////////////////////////////////////////////////////////////////////////
// PathConstraintPathHermite -> PathConstraintPath

OPAQUE_WRAPPER(JPC_PathConstraintPathHermite, JPH::PathConstraintPathHermite)

JPC_API JPC_PathConstraintPathHermite* JPC_PathConstraintPathHermite_new() {
	auto* path = new JPH::PathConstraintPathHermite();
	path->AddRef();
	return to_jpc(path);
}

JPC_API void JPC_PathConstraintPathHermite_AddPoint(
	JPC_PathConstraintPathHermite* self,
	JPC_Vec3 inPosition,
	JPC_Vec3 inTangent,
	JPC_Vec3 inNormal)
{
	to_jph(self)->AddPoint(to_jph(inPosition), to_jph(inTangent), to_jph(inNormal));
}

////////////////////////////////////////////////////////////////////////////////
// PathConstraint -> TwoBodyConstraint -> Constraint -> RefTarget<Constraint>

OPAQUE_WRAPPER(JPC_PathConstraint, JPH::PathConstraint);

JPC_API void JPC_PathConstraint_SetPath(
	JPC_PathConstraint* self,
	JPC_PathConstraintPath* inPath,
	float inPathFraction)
{
	to_jph(self)->SetPath(to_jph(inPath), inPathFraction);
}

JPC_API float JPC_PathConstraint_GetPathFraction(const JPC_PathConstraint* self) {
	return to_jph(self)->GetPathFraction();
}

JPC_API void JPC_PathConstraint_SetMaxFrictionForce(JPC_PathConstraint* self, float inFrictionForce) {
	to_jph(self)->SetMaxFrictionForce(inFrictionForce);
}

JPC_API float JPC_PathConstraint_GetMaxFrictionForce(const JPC_PathConstraint* self) {
	return to_jph(self)->GetMaxFrictionForce();
}

JPC_API void JPC_PathConstraint_SetPositionMotorState(JPC_PathConstraint* self, JPC_MotorState inState) {
	to_jph(self)->SetPositionMotorState(to_jph(inState));
}

JPC_API JPC_MotorState JPC_PathConstraint_GetPositionMotorState(const JPC_PathConstraint* self) {
	return to_jpc(to_jph(self)->GetPositionMotorState());
}

JPC_API void JPC_PathConstraint_SetTargetVelocity(JPC_PathConstraint* self, float inVelocity) {
	to_jph(self)->SetTargetVelocity(inVelocity);
}

JPC_API float JPC_PathConstraint_GetTargetVelocity(const JPC_PathConstraint* self) {
	return to_jph(self)->GetTargetVelocity();
}

JPC_API void JPC_PathConstraint_SetTargetPathFraction(JPC_PathConstraint* self, float inFraction) {
	to_jph(self)->SetTargetPathFraction(inFraction);
}

JPC_API float JPC_PathConstraint_GetTargetPathFraction(const JPC_PathConstraint* self) {
	return to_jph(self)->GetTargetPathFraction();
}

JPC_API JPC_Vec2 JPC_PathConstraint_GetTotalLambdaPosition(const JPC_PathConstraint* self) {
	return to_jpc(to_jph(self)->GetTotalLambdaPosition());
}

JPC_API float JPC_PathConstraint_GetTotalLambdaPositionLimits(const JPC_PathConstraint* self) {
	return to_jph(self)->GetTotalLambdaPositionLimits();
}

JPC_API float JPC_PathConstraint_GetTotalLambdaMotor(const JPC_PathConstraint* self) {
	return to_jph(self)->GetTotalLambdaMotor();
}

JPC_API JPC_Vec3 JPC_PathConstraint_GetTotalLambdaRotation(const JPC_PathConstraint* self) {
	return to_jpc(to_jph(self)->GetTotalLambdaRotation());
}

JPC_IMPL void JPC_PathConstraintSettings_to_jpc(
	JPC_PathConstraintSettings* outJpc,
	const JPH::PathConstraintSettings* inJph)
{
	JPC_ConstraintSettings_to_jpc(&outJpc->ConstraintSettings, inJph);

	outJpc->Path = const_cast<JPC_PathConstraintPath*>(to_jpc(inJph->mPath.GetPtr()));
	outJpc->PathPosition = to_jpc(inJph->mPathPosition);
	outJpc->PathRotation = to_jpc(inJph->mPathRotation);
	outJpc->PathFraction = inJph->mPathFraction;
	outJpc->MaxFrictionForce = inJph->mMaxFrictionForce;
	JPC_MotorSettings_to_jpc(&outJpc->PositionMotorSettings, &inJph->mPositionMotorSettings);
	outJpc->RotationConstraintType = static_cast<JPC_PathRotationConstraintType>(inJph->mRotationConstraintType);
}

JPC_IMPL void JPC_PathConstraintSettings_to_jph(
	const JPC_PathConstraintSettings* inJpc,
	JPH::PathConstraintSettings* outJph)
{
	JPC_ConstraintSettings_to_jph(&inJpc->ConstraintSettings, outJph);

	outJph->mPath = to_jph(inJpc->Path);
	outJph->mPathPosition = to_jph(inJpc->PathPosition);
	outJph->mPathRotation = to_jph(inJpc->PathRotation);
	outJph->mPathFraction = inJpc->PathFraction;
	outJph->mMaxFrictionForce = inJpc->MaxFrictionForce;
	JPC_MotorSettings_to_jph(&inJpc->PositionMotorSettings, &outJph->mPositionMotorSettings);
	outJph->mRotationConstraintType = static_cast<JPH::EPathRotationConstraintType>(inJpc->RotationConstraintType);
}

JPC_API void JPC_PathConstraintSettings_default(JPC_PathConstraintSettings* settings) {
	JPH::PathConstraintSettings defaultSettings{};
	JPC_PathConstraintSettings_to_jpc(settings, &defaultSettings);
}

JPC_API JPC_PathConstraint* JPC_PathConstraintSettings_Create(
	const JPC_PathConstraintSettings* self,
	JPC_Body* inBody1,
	JPC_Body* inBody2)
{
	JPH::PathConstraintSettings jphSettings;
	JPC_PathConstraintSettings_to_jph(self, &jphSettings);

	JPH::PathConstraint* outJph = new JPH::PathConstraint(*to_jph(inBody1), *to_jph(inBody2), jphSettings);
	return (JPC_PathConstraint*)outJph;
}

////////////////////////////////////////////////////////////////////////////////
// PhysicsMaterial

OPAQUE_WRAPPER(JPC_PhysicsMaterial, JPH::PhysicsMaterial)

JPC_API uint32_t JPC_PhysicsMaterial_GetRefCount(const JPC_PhysicsMaterial* self) {
	return to_jph(self)->GetRefCount();
}

JPC_API void JPC_PhysicsMaterial_AddRef(const JPC_PhysicsMaterial* self) {
	to_jph(self)->AddRef();
}

JPC_API void JPC_PhysicsMaterial_Release(const JPC_PhysicsMaterial* self) {
	to_jph(self)->Release();
}

JPC_API const char* JPC_PhysicsMaterial_GetDebugName(const JPC_PhysicsMaterial* self) {
	return to_jph(self)->GetDebugName();
}

////////////////////////////////////////////////////////////////////////////////
// JPC_CharacterContactListenerBridge

class JPC_CharacterContactListenerBridge final : public JPH::CharacterContactListener {
public:
	explicit JPC_CharacterContactListenerBridge(void* self, JPC_CharacterContactListenerFns fns)
		: self(self), fns(fns) {}

	void OnAdjustBodyVelocity(
		const JPH::CharacterVirtual* inCharacter,
		const JPH::Body& inBody2,
		JPH::Vec3& ioLinearVelocity,
		JPH::Vec3& ioAngularVelocity) override
	{
		if (fns.OnAdjustBodyVelocity == nullptr) return;
		JPC_Vec3 linear = to_jpc(ioLinearVelocity);
		JPC_Vec3 angular = to_jpc(ioAngularVelocity);
		fns.OnAdjustBodyVelocity(self, to_jpc(inCharacter), to_jpc(&inBody2), &linear, &angular);
		ioLinearVelocity = to_jph(linear);
		ioAngularVelocity = to_jph(angular);
	}

	bool OnContactValidate(
		const JPH::CharacterVirtual* inCharacter,
		const JPH::BodyID& inBodyID2,
		const JPH::SubShapeID& inSubShapeID2) override
	{
		if (fns.OnContactValidate == nullptr) return true;
		return fns.OnContactValidate(self, to_jpc(inCharacter), to_jpc(inBodyID2), to_jpc(inSubShapeID2));
	}

	void OnContactAdded(
		const JPH::CharacterVirtual* inCharacter,
		const JPH::BodyID& inBodyID2,
		const JPH::SubShapeID& inSubShapeID2,
		JPH::RVec3Arg inContactPosition,
		JPH::Vec3Arg inContactNormal,
		JPH::CharacterContactSettings& ioSettings) override
	{
		if (fns.OnContactAdded == nullptr) return;
		JPC_CharacterContactSettings settings = {
			ioSettings.mCanPushCharacter,
			ioSettings.mCanReceiveImpulses
		};
		fns.OnContactAdded(
			self,
			to_jpc(inCharacter),
			to_jpc(inBodyID2),
			to_jpc(inSubShapeID2),
			to_jpc(inContactPosition),
			to_jpc(inContactNormal),
			&settings);
		ioSettings.mCanPushCharacter = settings.CanPushCharacter;
		ioSettings.mCanReceiveImpulses = settings.CanReceiveImpulses;
	}

	void OnContactPersisted(
		const JPH::CharacterVirtual* inCharacter,
		const JPH::BodyID& inBodyID2,
		const JPH::SubShapeID& inSubShapeID2,
		JPH::RVec3Arg inContactPosition,
		JPH::Vec3Arg inContactNormal,
		JPH::CharacterContactSettings& ioSettings) override
	{
		if (fns.OnContactPersisted == nullptr) return;
		JPC_CharacterContactSettings settings = {
			ioSettings.mCanPushCharacter,
			ioSettings.mCanReceiveImpulses
		};
		fns.OnContactPersisted(
			self,
			to_jpc(inCharacter),
			to_jpc(inBodyID2),
			to_jpc(inSubShapeID2),
			to_jpc(inContactPosition),
			to_jpc(inContactNormal),
			&settings);
		ioSettings.mCanPushCharacter = settings.CanPushCharacter;
		ioSettings.mCanReceiveImpulses = settings.CanReceiveImpulses;
	}

	void OnContactSolve(
		const JPH::CharacterVirtual* inCharacter,
		const JPH::BodyID& inBodyID2,
		const JPH::SubShapeID& inSubShapeID2,
		JPH::RVec3Arg inContactPosition,
		JPH::Vec3Arg inContactNormal,
		JPH::Vec3Arg inContactVelocity,
		const JPH::PhysicsMaterial* inContactMaterial,
		JPH::Vec3Arg inCharacterVelocity,
		JPH::Vec3& ioNewCharacterVelocity) override
	{
		if (fns.OnContactSolve == nullptr) return;
		JPC_Vec3 newVelocity = to_jpc(ioNewCharacterVelocity);
		fns.OnContactSolve(
			self,
			to_jpc(inCharacter),
			to_jpc(inBodyID2),
			to_jpc(inSubShapeID2),
			to_jpc(inContactPosition),
			to_jpc(inContactNormal),
			to_jpc(inContactVelocity),
			to_jpc(inContactMaterial),
			to_jpc(inCharacterVelocity),
			&newVelocity);
		ioNewCharacterVelocity = to_jph(newVelocity);
	}

private:
	void* self;
	JPC_CharacterContactListenerFns fns;
};

OPAQUE_WRAPPER(JPC_CharacterContactListener, JPC_CharacterContactListenerBridge)
DESTRUCTOR(JPC_CharacterContactListener)

JPC_API JPC_CharacterContactListener* JPC_CharacterContactListener_new(
	void* self,
	JPC_CharacterContactListenerFns fns)
{
	return to_jpc(new JPC_CharacterContactListenerBridge(self, fns));
}

JPC_API void JPC_CharacterVirtual_SetListener(
	JPC_CharacterVirtual* self,
	JPC_CharacterContactListener* inListener)
{
	to_jph(self)->SetListener(to_jph(inListener));
}

JPC_API JPC_CharacterContactListener* JPC_CharacterVirtual_GetListener(
	const JPC_CharacterVirtual* self)
{
	return to_jpc(static_cast<JPC_CharacterContactListenerBridge*>(to_jph(self)->GetListener()));
}

////////////////////////////////////////////////////////////////////////////////
// VehicleEngineSettings

JPC_API void JPC_VehicleEngineSettings_default(JPC_VehicleEngineSettings* object) {
	JPH::VehicleEngineSettings d;
	object->MaxTorque      = d.mMaxTorque;
	object->MinRPM         = d.mMinRPM;
	object->MaxRPM         = d.mMaxRPM;
	object->Inertia        = d.mInertia;
	object->AngularDamping = d.mAngularDamping;
}

////////////////////////////////////////////////////////////////////////////////
// VehicleTransmissionSettings

JPC_API void JPC_VehicleTransmissionSettings_default(JPC_VehicleTransmissionSettings* object) {
	JPH::VehicleTransmissionSettings d;
	object->Mode              = static_cast<JPC_VehicleTransmissionMode>(d.mMode);
	object->SwitchUpRPM       = d.mShiftUpRPM;
	object->SwitchDownRPM     = d.mShiftDownRPM;
	object->SwitchTime        = d.mSwitchTime;
	object->ClutchReleaseTime = d.mClutchReleaseTime;
	object->SwitchLatency     = d.mSwitchLatency;
	object->ClutchStrength    = d.mClutchStrength;

	object->GearRatiosLen = (uint)std::min(d.mGearRatios.size(), (size_t)10);
	for (uint i = 0; i < object->GearRatiosLen; i++)
		object->GearRatios[i] = d.mGearRatios[i];

	object->ReverseGearRatiosLen = (uint)std::min(d.mReverseGearRatios.size(), (size_t)4);
	for (uint i = 0; i < object->ReverseGearRatiosLen; i++)
		object->ReverseGearRatios[i] = d.mReverseGearRatios[i];
}

////////////////////////////////////////////////////////////////////////////////
// VehicleDifferentialSettings

JPC_API void JPC_VehicleDifferentialSettings_default(JPC_VehicleDifferentialSettings* object) {
	JPH::VehicleDifferentialSettings d;
	object->LeftWheel          = d.mLeftWheel;
	object->RightWheel         = d.mRightWheel;
	object->DifferentialRatio  = d.mDifferentialRatio;
	object->LeftRightSplit     = d.mLeftRightSplit;
	object->LimitedSlipRatio   = d.mLimitedSlipRatio;
	object->EngineTorqueRatio  = d.mEngineTorqueRatio;
}

////////////////////////////////////////////////////////////////////////////////
// WheelSettingsWV

OPAQUE_WRAPPER(JPC_WheelSettingsWV, JPH::WheelSettingsWV)

JPC_API JPC_WheelSettingsWV* JPC_WheelSettingsWV_new() {
	auto* p = new JPH::WheelSettingsWV();
	p->AddRef();
	return to_jpc(p);
}

JPC_API void JPC_WheelSettingsWV_delete(JPC_WheelSettingsWV* self) {
	to_jph(self)->Release();
}

JPC_API void JPC_WheelSettingsWV_SetPosition(JPC_WheelSettingsWV* self, JPC_Vec3 inPosition) {
	to_jph(self)->mPosition = to_jph(inPosition);
}

JPC_API JPC_Vec3 JPC_WheelSettingsWV_GetPosition(const JPC_WheelSettingsWV* self) {
	return to_jpc(to_jph(self)->mPosition);
}

JPC_API void JPC_WheelSettingsWV_SetSuspensionDirection(JPC_WheelSettingsWV* self, JPC_Vec3 inDirection) {
	to_jph(self)->mSuspensionDirection = to_jph(inDirection);
}

JPC_API JPC_Vec3 JPC_WheelSettingsWV_GetSuspensionDirection(const JPC_WheelSettingsWV* self) {
	return to_jpc(to_jph(self)->mSuspensionDirection);
}

JPC_API void JPC_WheelSettingsWV_SetSteeringAxis(JPC_WheelSettingsWV* self, JPC_Vec3 inAxis) {
	to_jph(self)->mSteeringAxis = to_jph(inAxis);
}

JPC_API JPC_Vec3 JPC_WheelSettingsWV_GetSteeringAxis(const JPC_WheelSettingsWV* self) {
	return to_jpc(to_jph(self)->mSteeringAxis);
}

JPC_API void JPC_WheelSettingsWV_SetWheelUp(JPC_WheelSettingsWV* self, JPC_Vec3 inUp) {
	to_jph(self)->mWheelUp = to_jph(inUp);
}

JPC_API JPC_Vec3 JPC_WheelSettingsWV_GetWheelUp(const JPC_WheelSettingsWV* self) {
	return to_jpc(to_jph(self)->mWheelUp);
}

JPC_API void JPC_WheelSettingsWV_SetWheelForward(JPC_WheelSettingsWV* self, JPC_Vec3 inForward) {
	to_jph(self)->mWheelForward = to_jph(inForward);
}

JPC_API JPC_Vec3 JPC_WheelSettingsWV_GetWheelForward(const JPC_WheelSettingsWV* self) {
	return to_jpc(to_jph(self)->mWheelForward);
}

JPC_API void JPC_WheelSettingsWV_SetSuspensionMinLength(JPC_WheelSettingsWV* self, float inLength) {
	to_jph(self)->mSuspensionMinLength = inLength;
}

JPC_API float JPC_WheelSettingsWV_GetSuspensionMinLength(const JPC_WheelSettingsWV* self) {
	return to_jph(self)->mSuspensionMinLength;
}

JPC_API void JPC_WheelSettingsWV_SetSuspensionMaxLength(JPC_WheelSettingsWV* self, float inLength) {
	to_jph(self)->mSuspensionMaxLength = inLength;
}

JPC_API float JPC_WheelSettingsWV_GetSuspensionMaxLength(const JPC_WheelSettingsWV* self) {
	return to_jph(self)->mSuspensionMaxLength;
}

JPC_API void JPC_WheelSettingsWV_SetSuspensionPreloadLength(JPC_WheelSettingsWV* self, float inLength) {
	to_jph(self)->mSuspensionPreloadLength = inLength;
}

JPC_API float JPC_WheelSettingsWV_GetSuspensionPreloadLength(const JPC_WheelSettingsWV* self) {
	return to_jph(self)->mSuspensionPreloadLength;
}

JPC_API void JPC_WheelSettingsWV_SetRadius(JPC_WheelSettingsWV* self, float inRadius) {
	to_jph(self)->mRadius = inRadius;
}

JPC_API float JPC_WheelSettingsWV_GetRadius(const JPC_WheelSettingsWV* self) {
	return to_jph(self)->mRadius;
}

JPC_API void JPC_WheelSettingsWV_SetWidth(JPC_WheelSettingsWV* self, float inWidth) {
	to_jph(self)->mWidth = inWidth;
}

JPC_API float JPC_WheelSettingsWV_GetWidth(const JPC_WheelSettingsWV* self) {
	return to_jph(self)->mWidth;
}

JPC_API void JPC_WheelSettingsWV_SetMaxSteerAngle(JPC_WheelSettingsWV* self, float inAngle) {
	to_jph(self)->mMaxSteerAngle = inAngle;
}

JPC_API float JPC_WheelSettingsWV_GetMaxSteerAngle(const JPC_WheelSettingsWV* self) {
	return to_jph(self)->mMaxSteerAngle;
}

JPC_API void JPC_WheelSettingsWV_SetMaxBrakeTorque(JPC_WheelSettingsWV* self, float inTorque) {
	to_jph(self)->mMaxBrakeTorque = inTorque;
}

JPC_API float JPC_WheelSettingsWV_GetMaxBrakeTorque(const JPC_WheelSettingsWV* self) {
	return to_jph(self)->mMaxBrakeTorque;
}

JPC_API void JPC_WheelSettingsWV_SetMaxHandBrakeTorque(JPC_WheelSettingsWV* self, float inTorque) {
	to_jph(self)->mMaxHandBrakeTorque = inTorque;
}

JPC_API float JPC_WheelSettingsWV_GetMaxHandBrakeTorque(const JPC_WheelSettingsWV* self) {
	return to_jph(self)->mMaxHandBrakeTorque;
}

////////////////////////////////////////////////////////////////////////////////
// WheeledVehicleControllerSettings

OPAQUE_WRAPPER(JPC_WheeledVehicleControllerSettings, JPH::WheeledVehicleControllerSettings)

JPC_API JPC_WheeledVehicleControllerSettings* JPC_WheeledVehicleControllerSettings_new() {
	return to_jpc(new JPH::WheeledVehicleControllerSettings());
}

JPC_API void JPC_WheeledVehicleControllerSettings_delete(JPC_WheeledVehicleControllerSettings* self) {
	delete to_jph(self);
}

JPC_API void JPC_WheeledVehicleControllerSettings_SetEngine(
	JPC_WheeledVehicleControllerSettings* self,
	const JPC_VehicleEngineSettings* inEngine)
{
	JPH::VehicleEngineSettings& engine = to_jph(self)->mEngine;
	engine.mMaxTorque      = inEngine->MaxTorque;
	engine.mMinRPM         = inEngine->MinRPM;
	engine.mMaxRPM         = inEngine->MaxRPM;
	engine.mInertia        = inEngine->Inertia;
	engine.mAngularDamping = inEngine->AngularDamping;
}

JPC_API void JPC_WheeledVehicleControllerSettings_SetTransmission(
	JPC_WheeledVehicleControllerSettings* self,
	const JPC_VehicleTransmissionSettings* inTransmission)
{
	JPH::VehicleTransmissionSettings& t = to_jph(self)->mTransmission;
	t.mMode              = static_cast<JPH::ETransmissionMode>(inTransmission->Mode);
	t.mShiftUpRPM        = inTransmission->SwitchUpRPM;
	t.mShiftDownRPM      = inTransmission->SwitchDownRPM;
	t.mSwitchTime        = inTransmission->SwitchTime;
	t.mClutchReleaseTime = inTransmission->ClutchReleaseTime;
	t.mSwitchLatency     = inTransmission->SwitchLatency;
	t.mClutchStrength    = inTransmission->ClutchStrength;

	t.mGearRatios.clear();
	for (uint i = 0; i < inTransmission->GearRatiosLen; i++)
		t.mGearRatios.push_back(inTransmission->GearRatios[i]);

	t.mReverseGearRatios.clear();
	for (uint i = 0; i < inTransmission->ReverseGearRatiosLen; i++)
		t.mReverseGearRatios.push_back(inTransmission->ReverseGearRatios[i]);
}

JPC_API void JPC_WheeledVehicleControllerSettings_AddDifferential(
	JPC_WheeledVehicleControllerSettings* self,
	const JPC_VehicleDifferentialSettings* inDifferential)
{
	JPH::VehicleDifferentialSettings d;
	d.mLeftWheel         = inDifferential->LeftWheel;
	d.mRightWheel        = inDifferential->RightWheel;
	d.mDifferentialRatio = inDifferential->DifferentialRatio;
	d.mLeftRightSplit    = inDifferential->LeftRightSplit;
	d.mLimitedSlipRatio  = inDifferential->LimitedSlipRatio;
	d.mEngineTorqueRatio = inDifferential->EngineTorqueRatio;
	to_jph(self)->mDifferentials.push_back(d);
}

////////////////////////////////////////////////////////////////////////////////
// VehicleConstraintSettings

OPAQUE_WRAPPER(JPC_VehicleConstraintSettings, JPH::VehicleConstraintSettings)

JPC_API JPC_VehicleConstraintSettings* JPC_VehicleConstraintSettings_new() {
	return to_jpc(new JPH::VehicleConstraintSettings());
}

JPC_API void JPC_VehicleConstraintSettings_delete(JPC_VehicleConstraintSettings* self) {
	delete to_jph(self);
}

JPC_API void JPC_VehicleConstraintSettings_SetUp(JPC_VehicleConstraintSettings* self, JPC_Vec3 inUp) {
	to_jph(self)->mUp = to_jph(inUp);
}

JPC_API void JPC_VehicleConstraintSettings_SetForward(JPC_VehicleConstraintSettings* self, JPC_Vec3 inForward) {
	to_jph(self)->mForward = to_jph(inForward);
}

JPC_API void JPC_VehicleConstraintSettings_SetMaxPitchRollAngle(JPC_VehicleConstraintSettings* self, float inAngle) {
	to_jph(self)->mMaxPitchRollAngle = inAngle;
}

JPC_API void JPC_VehicleConstraintSettings_AddWheel(
	JPC_VehicleConstraintSettings* self,
	const JPC_WheelSettingsWV* inWheel)
{
	to_jph(self)->mWheels.push_back(const_cast<JPH::WheelSettingsWV*>(to_jph(inWheel)));
}

JPC_API void JPC_VehicleConstraintSettings_AddAntiRollBar(
	JPC_VehicleConstraintSettings* self,
	JPC_VehicleAntiRollBar inBar)
{
	JPH::VehicleAntiRollBar bar;
	bar.mLeftWheel  = inBar.LeftWheel;
	bar.mRightWheel = inBar.RightWheel;
	bar.mStiffness  = inBar.Stiffness;
	to_jph(self)->mAntiRollBars.push_back(bar);
}

JPC_API void JPC_VehicleConstraintSettings_SetController(
	JPC_VehicleConstraintSettings* self,
	const JPC_WheeledVehicleControllerSettings* inController)
{
	to_jph(self)->mController = const_cast<JPH::WheeledVehicleControllerSettings*>(to_jph(inController));
}

OPAQUE_WRAPPER(JPC_VehicleConstraint, JPH::VehicleConstraint)

JPC_API JPC_VehicleConstraint* JPC_VehicleConstraintSettings_Create(
	const JPC_VehicleConstraintSettings* self,
	JPC_Body* inVehicleBody)
{
	JPH::VehicleConstraint* constraint = new JPH::VehicleConstraint(
		*to_jph(inVehicleBody),
		*to_jph(self));
	constraint->AddRef();
	return to_jpc(constraint);
}

JPC_API JPC_Constraint* JPC_VehicleConstraint_AsConstraint(JPC_VehicleConstraint* self) {
	return to_jpc(static_cast<JPH::Constraint*>(to_jph(self)));
}

////////////////////////////////////////////////////////////////////////////////
// VehicleConstraint runtime

OPAQUE_WRAPPER(JPC_WheeledVehicleController, JPH::WheeledVehicleController)

JPC_API JPC_WheeledVehicleController* JPC_VehicleConstraint_GetWheeledController(JPC_VehicleConstraint* self) {
	return to_jpc(static_cast<JPH::WheeledVehicleController*>(to_jph(self)->GetController()));
}

JPC_API void JPC_WheeledVehicleController_SetDriverInput(
	JPC_WheeledVehicleController* self,
	float inForward,
	float inRight,
	float inBrake,
	float inHandBrake)
{
	to_jph(self)->SetDriverInput(inForward, inRight, inBrake, inHandBrake);
}

////////////////////////////////////////////////////////////////////////////////
// VehicleCollisionTester

OPAQUE_WRAPPER(JPC_VehicleCollisionTester, JPH::VehicleCollisionTester)

JPC_API JPC_VehicleCollisionTester* JPC_VehicleCollisionTesterRay_new(
	JPC_ObjectLayer inObjectLayer,
	JPC_Vec3 inUp,
	float inMaxSlopeAngle)
{
	auto* tester = new JPH::VehicleCollisionTesterRay(inObjectLayer, to_jph(inUp), inMaxSlopeAngle);
	tester->AddRef();
	return to_jpc(static_cast<JPH::VehicleCollisionTester*>(tester));
}

JPC_API JPC_VehicleCollisionTester* JPC_VehicleCollisionTesterCastSphere_new(
	JPC_ObjectLayer inObjectLayer,
	float inRadius,
	JPC_Vec3 inUp,
	float inMaxSlopeAngle)
{
	auto* tester = new JPH::VehicleCollisionTesterCastSphere(inObjectLayer, inRadius, to_jph(inUp), inMaxSlopeAngle);
	tester->AddRef();
	return to_jpc(static_cast<JPH::VehicleCollisionTester*>(tester));
}

JPC_API JPC_VehicleCollisionTester* JPC_VehicleCollisionTesterCastCylinder_new(
	JPC_ObjectLayer inObjectLayer,
	float inConvexRadiusFraction)
{
	auto* tester = new JPH::VehicleCollisionTesterCastCylinder(inObjectLayer, inConvexRadiusFraction);
	tester->AddRef();
	return to_jpc(static_cast<JPH::VehicleCollisionTester*>(tester));
}

JPC_API void JPC_VehicleCollisionTester_delete(JPC_VehicleCollisionTester* self) {
	to_jph(self)->Release();
}

JPC_API void JPC_VehicleConstraint_SetVehicleCollisionTester(
	JPC_VehicleConstraint* self,
	JPC_VehicleCollisionTester* inTester)
{
	to_jph(self)->SetVehicleCollisionTester(to_jph(inTester));
}

////////////////////////////////////////////////////////////////////////////////
// PhysicsSystem step listener

JPC_API void JPC_PhysicsSystem_AddStepListener(JPC_PhysicsSystem* self, JPC_VehicleConstraint* inConstraint) {
	to_jph(self)->AddStepListener(to_jph(inConstraint));
}

JPC_API void JPC_PhysicsSystem_RemoveStepListener(JPC_PhysicsSystem* self, JPC_VehicleConstraint* inConstraint) {
	to_jph(self)->RemoveStepListener(to_jph(inConstraint));
}

////////////////////////////////////////////////////////////////////////////////
// SoftBodySharedSettings

OPAQUE_WRAPPER(JPC_SoftBodySharedSettings, JPH::SoftBodySharedSettings)

JPC_API JPC_SoftBodySharedSettings* JPC_SoftBodySharedSettings_new() {
	auto* settings = new JPH::SoftBodySharedSettings();
	settings->AddRef();
	return to_jpc(settings);
}

JPC_API uint32_t JPC_SoftBodySharedSettings_GetRefCount(const JPC_SoftBodySharedSettings* self) {
	return to_jph(self)->GetRefCount();
}

JPC_API void JPC_SoftBodySharedSettings_AddRef(const JPC_SoftBodySharedSettings* self) {
	to_jph(self)->AddRef();
}

JPC_API void JPC_SoftBodySharedSettings_Release(const JPC_SoftBodySharedSettings* self) {
	to_jph(self)->Release();
}

JPC_API void JPC_SoftBodySharedSettings_AddVertex(JPC_SoftBodySharedSettings* self, const JPC_SoftBodyVertex* inVertex) {
	JPH::SoftBodySharedSettings::Vertex v;
	v.mPosition = JPH::Float3(inVertex->Position.x, inVertex->Position.y, inVertex->Position.z);
	v.mVelocity = JPH::Float3(inVertex->Velocity.x, inVertex->Velocity.y, inVertex->Velocity.z);
	v.mInvMass = inVertex->InvMass;
	to_jph(self)->mVertices.push_back(v);
}

JPC_API void JPC_SoftBodySharedSettings_AddEdgeConstraint(JPC_SoftBodySharedSettings* self, const JPC_SoftBodyEdgeConstraint* inEdge) {
	JPH::SoftBodySharedSettings::Edge e;
	e.mVertex[0] = inEdge->Vertex0;
	e.mVertex[1] = inEdge->Vertex1;
	e.mCompliance = inEdge->Compliance;
	to_jph(self)->mEdgeConstraints.push_back(e);
}

JPC_API void JPC_SoftBodySharedSettings_AddFace(JPC_SoftBodySharedSettings* self, const JPC_SoftBodyFace* inFace) {
	JPH::SoftBodySharedSettings::Face f;
	f.mVertex[0] = inFace->Vertex[0];
	f.mVertex[1] = inFace->Vertex[1];
	f.mVertex[2] = inFace->Vertex[2];
	to_jph(self)->mFaces.push_back(f);
}

JPC_API void JPC_SoftBodySharedSettings_Optimize(JPC_SoftBodySharedSettings* self) {
	to_jph(self)->Optimize();
}

////////////////////////////////////////////////////////////////////////////////
// SoftBodyCreationSettings

JPC_IMPL JPH::SoftBodyCreationSettings JPC_SoftBodyCreationSettings_to_jph(const JPC_SoftBodyCreationSettings* settings) {
	JPH::SoftBodyCreationSettings output{};

	output.mSettings = to_jph(settings->Settings);
	output.mPosition = to_jph(settings->Position);
	output.mRotation = to_jph(settings->Rotation);
	output.mUserData = settings->UserData;
	output.mObjectLayer = settings->ObjectLayer;
	output.mNumIterations = settings->NumIterations;
	output.mLinearDamping = settings->LinearDamping;
	output.mMaxLinearVelocity = settings->MaxLinearVelocity;
	output.mRestitution = settings->Restitution;
	output.mFriction = settings->Friction;
	output.mPressure = settings->Pressure;
	output.mGravityFactor = settings->GravityFactor;
	output.mVertexRadius = settings->VertexRadius;
	output.mUpdatePosition = settings->UpdatePosition;
	output.mMakeRotationIdentity = settings->MakeRotationIdentity;
	output.mAllowSleeping = settings->AllowSleeping;
	output.mFacesDoubleSided = settings->FacesDoubleSided;

	return output;
}

JPC_API void JPC_SoftBodyCreationSettings_default(JPC_SoftBodyCreationSettings* object) {
	JPH::SoftBodyCreationSettings defaults{};

	object->Settings = nullptr;
	object->Position = to_jpc(defaults.mPosition);
	object->Rotation = to_jpc(defaults.mRotation);
	object->UserData = defaults.mUserData;
	object->ObjectLayer = defaults.mObjectLayer;
	object->NumIterations = defaults.mNumIterations;
	object->LinearDamping = defaults.mLinearDamping;
	object->MaxLinearVelocity = defaults.mMaxLinearVelocity;
	object->Restitution = defaults.mRestitution;
	object->Friction = defaults.mFriction;
	object->Pressure = defaults.mPressure;
	object->GravityFactor = defaults.mGravityFactor;
	object->VertexRadius = defaults.mVertexRadius;
	object->UpdatePosition = defaults.mUpdatePosition;
	object->MakeRotationIdentity = defaults.mMakeRotationIdentity;
	object->AllowSleeping = defaults.mAllowSleeping;
	object->FacesDoubleSided = defaults.mFacesDoubleSided;
}

////////////////////////////////////////////////////////////////////////////////
// BodyInterface soft body methods

JPC_API JPC_Body* JPC_BodyInterface_CreateSoftBody(JPC_BodyInterface* self, const JPC_SoftBodyCreationSettings* inSettings) {
	return to_jpc(to_jph(self)->CreateSoftBody(JPC_SoftBodyCreationSettings_to_jph(inSettings)));
}

JPC_API JPC_Body* JPC_BodyInterface_CreateSoftBodyWithID(JPC_BodyInterface* self, JPC_BodyID inBodyID, const JPC_SoftBodyCreationSettings* inSettings) {
	return to_jpc(to_jph(self)->CreateSoftBodyWithID(to_jph(inBodyID), JPC_SoftBodyCreationSettings_to_jph(inSettings)));
}

JPC_API JPC_BodyID JPC_BodyInterface_CreateAndAddSoftBody(JPC_BodyInterface* self, const JPC_SoftBodyCreationSettings* inSettings, JPC_Activation inActivationMode) {
	return to_jpc(to_jph(self)->CreateAndAddSoftBody(JPC_SoftBodyCreationSettings_to_jph(inSettings), to_jph(inActivationMode)));
}

////////////////////////////////////////////////////////////////////////////////
// StateRecorder (wraps StateRecorderImpl)

struct JPC_StateRecorderWrapper {
	JPH::StateRecorderImpl recorder;
	std::string cached_data;
};

OPAQUE_WRAPPER(JPC_StateRecorder, JPC_StateRecorderWrapper)
DESTRUCTOR(JPC_StateRecorder)

JPC_API JPC_StateRecorder* JPC_StateRecorder_new() {
	return to_jpc(new JPC_StateRecorderWrapper());
}

JPC_API void JPC_StateRecorder_Clear(JPC_StateRecorder* self) {
	to_jph(self)->recorder.Clear();
}

JPC_API void JPC_StateRecorder_Rewind(JPC_StateRecorder* self) {
	to_jph(self)->recorder.Rewind();
}

JPC_API void JPC_StateRecorder_GetData(const JPC_StateRecorder* self, const uint8_t** outData, size_t* outLen) {
	JPC_StateRecorderWrapper* wrapper = to_jph(const_cast<JPC_StateRecorder*>(self));
	wrapper->cached_data = wrapper->recorder.GetData();
	*outData = reinterpret_cast<const uint8_t*>(wrapper->cached_data.data());
	*outLen = wrapper->cached_data.size();
}

JPC_API size_t JPC_StateRecorder_GetDataSize(const JPC_StateRecorder* self) {
	return to_jph(const_cast<JPC_StateRecorder*>(self))->recorder.GetDataSize();
}

////////////////////////////////////////////////////////////////////////////////
// PhysicsSystem save/restore state

JPC_API void JPC_PhysicsSystem_SaveState(JPC_PhysicsSystem* self, JPC_StateRecorder* inStateRecorder) {
	to_jph(self)->SaveState(to_jph(inStateRecorder)->recorder);
}

JPC_API bool JPC_PhysicsSystem_RestoreState(JPC_PhysicsSystem* self, JPC_StateRecorder* inStateRecorder) {
	return to_jph(self)->RestoreState(to_jph(inStateRecorder)->recorder);
}

////////////////////////////////////////////////////////////////////////////////
// RagdollSettings

OPAQUE_WRAPPER(JPC_Ragdoll, JPH::Ragdoll)
OPAQUE_WRAPPER(JPC_RagdollSettings, JPH::RagdollSettings)

JPC_API JPC_RagdollSettings* JPC_RagdollSettings_new() {
	auto* settings = new JPH::RagdollSettings();
	settings->AddRef();
	return to_jpc(settings);
}

JPC_API void JPC_RagdollSettings_delete(JPC_RagdollSettings* self) {
	delete to_jph(self);
}

JPC_API uint32_t JPC_RagdollSettings_GetRefCount(const JPC_RagdollSettings* self) {
	return to_jph(self)->GetRefCount();
}

JPC_API void JPC_RagdollSettings_AddRef(const JPC_RagdollSettings* self) {
	to_jph(self)->AddRef();
}

JPC_API void JPC_RagdollSettings_Release(const JPC_RagdollSettings* self) {
	to_jph(self)->Release();
}

JPC_API uint JPC_RagdollSettings_GetPartCount(const JPC_RagdollSettings* self) {
	return static_cast<uint>(to_jph(self)->mParts.size());
}

JPC_API void JPC_RagdollSettings_AddPart(JPC_RagdollSettings* self, const JPC_BodyCreationSettings* inBodyCreationSettings) {
	JPH::RagdollSettings::Part part;

	part.mPosition = to_jph(inBodyCreationSettings->Position);
	part.mRotation = to_jph(inBodyCreationSettings->Rotation);
	part.mLinearVelocity = to_jph(inBodyCreationSettings->LinearVelocity);
	part.mAngularVelocity = to_jph(inBodyCreationSettings->AngularVelocity);
	part.mUserData = inBodyCreationSettings->UserData;
	part.mObjectLayer = inBodyCreationSettings->ObjectLayer;
	part.mMotionType = to_jph(inBodyCreationSettings->MotionType);
	part.mAllowedDOFs = to_jph(inBodyCreationSettings->AllowedDOFs);
	part.mAllowDynamicOrKinematic = inBodyCreationSettings->AllowDynamicOrKinematic;
	part.mIsSensor = inBodyCreationSettings->IsSensor;
	part.mCollideKinematicVsNonDynamic = inBodyCreationSettings->CollideKinematicVsNonDynamic;
	part.mUseManifoldReduction = inBodyCreationSettings->UseManifoldReduction;
	part.mApplyGyroscopicForce = inBodyCreationSettings->ApplyGyroscopicForce;
	part.mMotionQuality = to_jph(inBodyCreationSettings->MotionQuality);
	part.mEnhancedInternalEdgeRemoval = inBodyCreationSettings->EnhancedInternalEdgeRemoval;
	part.mAllowSleeping = inBodyCreationSettings->AllowSleeping;
	part.mFriction = inBodyCreationSettings->Friction;
	part.mRestitution = inBodyCreationSettings->Restitution;
	part.mLinearDamping = inBodyCreationSettings->LinearDamping;
	part.mAngularDamping = inBodyCreationSettings->AngularDamping;
	part.mMaxLinearVelocity = inBodyCreationSettings->MaxLinearVelocity;
	part.mMaxAngularVelocity = inBodyCreationSettings->MaxAngularVelocity;
	part.mGravityFactor = inBodyCreationSettings->GravityFactor;
	part.mNumVelocityStepsOverride = inBodyCreationSettings->NumVelocityStepsOverride;
	part.mNumPositionStepsOverride = inBodyCreationSettings->NumPositionStepsOverride;
	part.mOverrideMassProperties = to_jph(inBodyCreationSettings->OverrideMassProperties);
	part.mInertiaMultiplier = inBodyCreationSettings->InertiaMultiplier;
	part.SetShape(to_jph(inBodyCreationSettings->Shape));
	part.mToParent = nullptr;

	to_jph(self)->mParts.push_back(part);
}

JPC_API JPC_Ragdoll* JPC_RagdollSettings_CreateRagdoll(
	const JPC_RagdollSettings* self,
	JPC_GroupID inCollisionGroup,
	uint64_t inUserData,
	JPC_PhysicsSystem* inPhysicsSystem)
{
	JPH::Ragdoll* ragdoll = to_jph(self)->CreateRagdoll(
		inCollisionGroup,
		inUserData,
		to_jph(inPhysicsSystem));
	return to_jpc(ragdoll);
}

////////////////////////////////////////////////////////////////////////////////
// Ragdoll runtime

JPC_API void JPC_Ragdoll_delete(JPC_Ragdoll* self) {
	delete to_jph(self);
}

JPC_API void JPC_Ragdoll_AddToPhysicsSystem(JPC_Ragdoll* self, JPC_Activation inActivationMode, bool inLockBodies) {
	to_jph(self)->AddToPhysicsSystem(to_jph(inActivationMode), inLockBodies);
}

JPC_API void JPC_Ragdoll_RemoveFromPhysicsSystem(JPC_Ragdoll* self, bool inLockBodies) {
	to_jph(self)->RemoveFromPhysicsSystem(inLockBodies);
}

JPC_API void JPC_Ragdoll_Activate(JPC_Ragdoll* self, bool inLockBodies) {
	to_jph(self)->Activate(inLockBodies);
}

JPC_API uint JPC_Ragdoll_GetBodyCount(const JPC_Ragdoll* self) {
	return static_cast<uint>(to_jph(self)->GetBodyCount());
}

JPC_API JPC_BodyID JPC_Ragdoll_GetBodyID(const JPC_Ragdoll* self, uint inBodyIndex) {
	return to_jpc(to_jph(self)->GetBodyID(static_cast<int>(inBodyIndex)));
}

JPC_API bool JPC_Ragdoll_IsActive(const JPC_Ragdoll* self, bool inLockBodies) {
	return to_jph(self)->IsActive(inLockBodies);
}

JPC_API void JPC_Ragdoll_ResetWarmStart(JPC_Ragdoll* self) {
	to_jph(self)->ResetWarmStart();
}

JPC_API void JPC_Ragdoll_SetGroupID(JPC_Ragdoll* self, JPC_GroupID inGroupID, bool inLockBodies) {
	to_jph(self)->SetGroupID(inGroupID, inLockBodies);
}

JPC_API void JPC_Ragdoll_SetLinearVelocity(JPC_Ragdoll* self, JPC_Vec3 inVelocity, bool inLockBodies) {
	to_jph(self)->SetLinearVelocity(to_jph(inVelocity), inLockBodies);
}

JPC_API void JPC_Ragdoll_AddLinearVelocity(JPC_Ragdoll* self, JPC_Vec3 inVelocity, bool inLockBodies) {
	to_jph(self)->AddLinearVelocity(to_jph(inVelocity), inLockBodies);
}

JPC_API void JPC_Ragdoll_AddImpulse(JPC_Ragdoll* self, JPC_Vec3 inImpulse, bool inLockBodies) {
	to_jph(self)->AddImpulse(to_jph(inImpulse), inLockBodies);
}

JPC_API uint JPC_Ragdoll_GetConstraintCount(const JPC_Ragdoll* self) {
	return static_cast<uint>(to_jph(self)->GetConstraintCount());
}

JPC_API JPC_TwoBodyConstraint* JPC_Ragdoll_GetConstraint(JPC_Ragdoll* self, uint inConstraintIndex) {
	return to_jpc(to_jph(self)->GetConstraint(static_cast<int>(inConstraintIndex)));
}

JPC_API void JPC_Ragdoll_GetBodyIDs(const JPC_Ragdoll* self, JPC_BodyID* outBodyIDs, uint* outCount) {
	const auto& ids = to_jph(self)->GetBodyIDs();
	*outCount = static_cast<uint>(ids.size());
	for (uint i = 0; i < *outCount; i++)
		outBodyIDs[i] = to_jpc(ids[i]);
}

JPC_API void JPC_RagdollSettings_SetPartFixedConstraint(JPC_RagdollSettings* self, uint inPartIndex, const JPC_FixedConstraintSettings* inSettings) {
	auto* jphSettings = new JPH::FixedConstraintSettings();
	JPC_FixedConstraintSettings_to_jph(inSettings, jphSettings);
	to_jph(self)->mParts[inPartIndex].mToParent = jphSettings;
}

JPC_API void JPC_RagdollSettings_SetPartHingeConstraint(JPC_RagdollSettings* self, uint inPartIndex, const JPC_HingeConstraintSettings* inSettings) {
	auto* jphSettings = new JPH::HingeConstraintSettings();
	JPC_HingeConstraintSettings_to_jph(inSettings, jphSettings);
	to_jph(self)->mParts[inPartIndex].mToParent = jphSettings;
}

JPC_API void JPC_RagdollSettings_SetPartSliderConstraint(JPC_RagdollSettings* self, uint inPartIndex, const JPC_SliderConstraintSettings* inSettings) {
	auto* jphSettings = new JPH::SliderConstraintSettings();
	JPC_SliderConstraintSettings_to_jph(inSettings, jphSettings);
	to_jph(self)->mParts[inPartIndex].mToParent = jphSettings;
}

JPC_API void JPC_RagdollSettings_SetPartSwingTwistConstraint(JPC_RagdollSettings* self, uint inPartIndex, const JPC_SwingTwistConstraintSettings* inSettings) {
	auto* jphSettings = new JPH::SwingTwistConstraintSettings();
	JPC_SwingTwistConstraintSettings_to_jph(inSettings, jphSettings);
	to_jph(self)->mParts[inPartIndex].mToParent = jphSettings;
}

JPC_API void JPC_RagdollSettings_SetPartPointConstraint(JPC_RagdollSettings* self, uint inPartIndex, const JPC_PointConstraintSettings* inSettings) {
	auto* jphSettings = new JPH::PointConstraintSettings();
	JPC_PointConstraintSettings_to_jph(inSettings, jphSettings);
	to_jph(self)->mParts[inPartIndex].mToParent = jphSettings;
}

////////////////////////////////////////////////////////////////////////////////
// HingeConstraint runtime additions

JPC_API float JPC_HingeConstraint_GetCurrentAngle(const JPC_HingeConstraint* self) {
	return to_jph(self)->GetCurrentAngle();
}

JPC_API JPC_Vec3 JPC_HingeConstraint_GetLocalSpacePoint1(const JPC_HingeConstraint* self) {
	return to_jpc(to_jph(self)->GetLocalSpacePoint1());
}

JPC_API JPC_Vec3 JPC_HingeConstraint_GetLocalSpacePoint2(const JPC_HingeConstraint* self) {
	return to_jpc(to_jph(self)->GetLocalSpacePoint2());
}

JPC_API JPC_Vec3 JPC_HingeConstraint_GetLocalSpaceHingeAxis1(const JPC_HingeConstraint* self) {
	return to_jpc(to_jph(self)->GetLocalSpaceHingeAxis1());
}

JPC_API JPC_Vec3 JPC_HingeConstraint_GetLocalSpaceHingeAxis2(const JPC_HingeConstraint* self) {
	return to_jpc(to_jph(self)->GetLocalSpaceHingeAxis2());
}

JPC_API JPC_Vec3 JPC_HingeConstraint_GetLocalSpaceNormalAxis1(const JPC_HingeConstraint* self) {
	return to_jpc(to_jph(self)->GetLocalSpaceNormalAxis1());
}

JPC_API JPC_Vec3 JPC_HingeConstraint_GetLocalSpaceNormalAxis2(const JPC_HingeConstraint* self) {
	return to_jpc(to_jph(self)->GetLocalSpaceNormalAxis2());
}

JPC_API void JPC_HingeConstraint_SetMaxFrictionTorque(JPC_HingeConstraint* self, float inFrictionTorque) {
	to_jph(self)->SetMaxFrictionTorque(inFrictionTorque);
}

JPC_API float JPC_HingeConstraint_GetMaxFrictionTorque(const JPC_HingeConstraint* self) {
	return to_jph(self)->GetMaxFrictionTorque();
}

JPC_API JPC_MotorSettings JPC_HingeConstraint_GetMotorSettings(const JPC_HingeConstraint* self) {
	JPC_MotorSettings out;
	JPC_MotorSettings_to_jpc(&out, &to_jph(self)->GetMotorSettings());
	return out;
}

JPC_API void JPC_HingeConstraint_SetMotorSettings(JPC_HingeConstraint* self, const JPC_MotorSettings* inSettings) {
	JPC_MotorSettings_to_jph(inSettings, &to_jph(self)->GetMotorSettings());
}

JPC_API void JPC_HingeConstraint_SetTargetOrientationBS(JPC_HingeConstraint* self, JPC_Quat inOrientation) {
	to_jph(self)->SetTargetOrientationBS(to_jph(inOrientation));
}

JPC_API void JPC_HingeConstraint_SetLimits(JPC_HingeConstraint* self, float inLimitsMin, float inLimitsMax) {
	to_jph(self)->SetLimits(inLimitsMin, inLimitsMax);
}

JPC_API float JPC_HingeConstraint_GetLimitsMin(const JPC_HingeConstraint* self) {
	return to_jph(self)->GetLimitsMin();
}

JPC_API float JPC_HingeConstraint_GetLimitsMax(const JPC_HingeConstraint* self) {
	return to_jph(self)->GetLimitsMax();
}

JPC_API bool JPC_HingeConstraint_HasLimits(const JPC_HingeConstraint* self) {
	return to_jph(self)->HasLimits();
}

JPC_API JPC_SpringSettings JPC_HingeConstraint_GetLimitsSpringSettings(const JPC_HingeConstraint* self) {
	JPC_SpringSettings out;
	JPC_SpringSettings_to_jpc(&out, &to_jph(self)->GetLimitsSpringSettings());
	return out;
}

JPC_API void JPC_HingeConstraint_SetLimitsSpringSettings(JPC_HingeConstraint* self, const JPC_SpringSettings* inSettings) {
	JPH::SpringSettings jph;
	JPC_SpringSettings_to_jph(inSettings, &jph);
	to_jph(self)->SetLimitsSpringSettings(jph);
}

////////////////////////////////////////////////////////////////////////////////
// SliderConstraint runtime additions

JPC_API float JPC_SliderConstraint_GetCurrentPosition(const JPC_SliderConstraint* self) {
	return to_jph(self)->GetCurrentPosition();
}

JPC_API void JPC_SliderConstraint_SetMaxFrictionForce(JPC_SliderConstraint* self, float inFrictionForce) {
	to_jph(self)->SetMaxFrictionForce(inFrictionForce);
}

JPC_API float JPC_SliderConstraint_GetMaxFrictionForce(const JPC_SliderConstraint* self) {
	return to_jph(self)->GetMaxFrictionForce();
}

JPC_API JPC_MotorSettings JPC_SliderConstraint_GetMotorSettings(const JPC_SliderConstraint* self) {
	JPC_MotorSettings out;
	JPC_MotorSettings_to_jpc(&out, &to_jph(self)->GetMotorSettings());
	return out;
}

JPC_API void JPC_SliderConstraint_SetMotorSettings(JPC_SliderConstraint* self, const JPC_MotorSettings* inSettings) {
	JPC_MotorSettings_to_jph(inSettings, &to_jph(self)->GetMotorSettings());
}

JPC_API void JPC_SliderConstraint_SetLimits(JPC_SliderConstraint* self, float inLimitsMin, float inLimitsMax) {
	to_jph(self)->SetLimits(inLimitsMin, inLimitsMax);
}

JPC_API float JPC_SliderConstraint_GetLimitsMin(const JPC_SliderConstraint* self) {
	return to_jph(self)->GetLimitsMin();
}

JPC_API float JPC_SliderConstraint_GetLimitsMax(const JPC_SliderConstraint* self) {
	return to_jph(self)->GetLimitsMax();
}

JPC_API bool JPC_SliderConstraint_HasLimits(const JPC_SliderConstraint* self) {
	return to_jph(self)->HasLimits();
}

JPC_API JPC_SpringSettings JPC_SliderConstraint_GetLimitsSpringSettings(const JPC_SliderConstraint* self) {
	JPC_SpringSettings out;
	JPC_SpringSettings_to_jpc(&out, &to_jph(self)->GetLimitsSpringSettings());
	return out;
}

JPC_API void JPC_SliderConstraint_SetLimitsSpringSettings(JPC_SliderConstraint* self, const JPC_SpringSettings* inSettings) {
	JPH::SpringSettings jph;
	JPC_SpringSettings_to_jph(inSettings, &jph);
	to_jph(self)->SetLimitsSpringSettings(jph);
}

////////////////////////////////////////////////////////////////////////////////
// DistanceConstraint runtime additions

JPC_API void JPC_DistanceConstraint_SetDistance(JPC_DistanceConstraint* self, float inMinDistance, float inMaxDistance) {
	to_jph(self)->SetDistance(inMinDistance, inMaxDistance);
}

JPC_API float JPC_DistanceConstraint_GetMinDistance(const JPC_DistanceConstraint* self) {
	return to_jph(self)->GetMinDistance();
}

JPC_API float JPC_DistanceConstraint_GetMaxDistance(const JPC_DistanceConstraint* self) {
	return to_jph(self)->GetMaxDistance();
}

JPC_API JPC_SpringSettings JPC_DistanceConstraint_GetLimitsSpringSettings(const JPC_DistanceConstraint* self) {
	JPC_SpringSettings out;
	JPC_SpringSettings_to_jpc(&out, &to_jph(self)->GetLimitsSpringSettings());
	return out;
}

JPC_API void JPC_DistanceConstraint_SetLimitsSpringSettings(JPC_DistanceConstraint* self, const JPC_SpringSettings* inSettings) {
	JPH::SpringSettings jph;
	JPC_SpringSettings_to_jph(inSettings, &jph);
	to_jph(self)->SetLimitsSpringSettings(jph);
}

////////////////////////////////////////////////////////////////////////////////
// PointConstraint runtime additions

JPC_API void JPC_PointConstraint_SetPoint1(JPC_PointConstraint* self, JPC_ConstraintSpace inSpace, JPC_RVec3 inPoint) {
	to_jph(self)->SetPoint1(static_cast<JPH::EConstraintSpace>(inSpace), to_jph(inPoint));
}

JPC_API void JPC_PointConstraint_SetPoint2(JPC_PointConstraint* self, JPC_ConstraintSpace inSpace, JPC_RVec3 inPoint) {
	to_jph(self)->SetPoint2(static_cast<JPH::EConstraintSpace>(inSpace), to_jph(inPoint));
}

JPC_API JPC_Vec3 JPC_PointConstraint_GetLocalSpacePoint1(const JPC_PointConstraint* self) {
	return to_jpc(to_jph(self)->GetLocalSpacePoint1());
}

JPC_API JPC_Vec3 JPC_PointConstraint_GetLocalSpacePoint2(const JPC_PointConstraint* self) {
	return to_jpc(to_jph(self)->GetLocalSpacePoint2());
}

////////////////////////////////////////////////////////////////////////////////
// SixDOFConstraint runtime additions

JPC_API bool JPC_SixDOFConstraint_IsFixedAxis(const JPC_SixDOFConstraint* self, JPC_SixDOFConstraint_Axis inAxis) {
	return to_jph(self)->IsFixedAxis(static_cast<JPH::SixDOFConstraint::EAxis>(inAxis));
}

JPC_API JPC_SpringSettings JPC_SixDOFConstraint_GetLimitsSpringSettings(const JPC_SixDOFConstraint* self, JPC_SixDOFConstraint_Axis inAxis) {
	JPC_SpringSettings out;
	JPC_SpringSettings_to_jpc(&out, &to_jph(self)->GetLimitsSpringSettings(static_cast<JPH::SixDOFConstraint::EAxis>(inAxis)));
	return out;
}

JPC_API void JPC_SixDOFConstraint_SetLimitsSpringSettings(JPC_SixDOFConstraint* self, JPC_SixDOFConstraint_Axis inAxis, const JPC_SpringSettings* inSettings) {
	JPH::SpringSettings jph;
	JPC_SpringSettings_to_jph(inSettings, &jph);
	to_jph(self)->SetLimitsSpringSettings(static_cast<JPH::SixDOFConstraint::EAxis>(inAxis), jph);
}

JPC_API JPC_MotorSettings JPC_SixDOFConstraint_GetMotorSettings(const JPC_SixDOFConstraint* self, JPC_SixDOFConstraint_Axis inAxis) {
	JPC_MotorSettings out;
	JPC_MotorSettings_to_jpc(&out, &to_jph(self)->GetMotorSettings(static_cast<JPH::SixDOFConstraint::EAxis>(inAxis)));
	return out;
}

JPC_API void JPC_SixDOFConstraint_SetMotorSettings(JPC_SixDOFConstraint* self, JPC_SixDOFConstraint_Axis inAxis, const JPC_MotorSettings* inSettings) {
	JPC_MotorSettings_to_jph(inSettings, &to_jph(self)->GetMotorSettings(static_cast<JPH::SixDOFConstraint::EAxis>(inAxis)));
}

JPC_API void JPC_SixDOFConstraint_SetMotorState(JPC_SixDOFConstraint* self, JPC_SixDOFConstraint_Axis inAxis, JPC_MotorState inState) {
	to_jph(self)->SetMotorState(static_cast<JPH::SixDOFConstraint::EAxis>(inAxis), to_jph(inState));
}

JPC_API JPC_MotorState JPC_SixDOFConstraint_GetMotorState(const JPC_SixDOFConstraint* self, JPC_SixDOFConstraint_Axis inAxis) {
	return to_jpc(to_jph(self)->GetMotorState(static_cast<JPH::SixDOFConstraint::EAxis>(inAxis)));
}

////////////////////////////////////////////////////////////////////////////////
// SwingTwistConstraint runtime additions

JPC_API JPC_Vec3 JPC_SwingTwistConstraint_GetLocalSpacePosition1(const JPC_SwingTwistConstraint* self) {
	return to_jpc(to_jph(self)->GetLocalSpacePosition1());
}

JPC_API JPC_Vec3 JPC_SwingTwistConstraint_GetLocalSpacePosition2(const JPC_SwingTwistConstraint* self) {
	return to_jpc(to_jph(self)->GetLocalSpacePosition2());
}

JPC_API JPC_Quat JPC_SwingTwistConstraint_GetConstraintToBody1(const JPC_SwingTwistConstraint* self) {
	return to_jpc(to_jph(self)->GetConstraintToBody1());
}

JPC_API JPC_Quat JPC_SwingTwistConstraint_GetConstraintToBody2(const JPC_SwingTwistConstraint* self) {
	return to_jpc(to_jph(self)->GetConstraintToBody2());
}

JPC_API JPC_MotorSettings JPC_SwingTwistConstraint_GetSwingMotorSettings(const JPC_SwingTwistConstraint* self) {
	JPC_MotorSettings out;
	JPC_MotorSettings_to_jpc(&out, &to_jph(self)->GetSwingMotorSettings());
	return out;
}

JPC_API void JPC_SwingTwistConstraint_SetSwingMotorSettings(JPC_SwingTwistConstraint* self, const JPC_MotorSettings* inSettings) {
	JPC_MotorSettings_to_jph(inSettings, &to_jph(self)->GetSwingMotorSettings());
}

JPC_API JPC_MotorSettings JPC_SwingTwistConstraint_GetTwistMotorSettings(const JPC_SwingTwistConstraint* self) {
	JPC_MotorSettings out;
	JPC_MotorSettings_to_jpc(&out, &to_jph(self)->GetTwistMotorSettings());
	return out;
}

JPC_API void JPC_SwingTwistConstraint_SetTwistMotorSettings(JPC_SwingTwistConstraint* self, const JPC_MotorSettings* inSettings) {
	JPC_MotorSettings_to_jph(inSettings, &to_jph(self)->GetTwistMotorSettings());
}

JPC_API void JPC_SwingTwistConstraint_SetTargetOrientationBS(JPC_SwingTwistConstraint* self, JPC_Quat inOrientation) {
	to_jph(self)->SetTargetOrientationBS(to_jph(inOrientation));
}

////////////////////////////////////////////////////////////////////////////////
// PathConstraint runtime additions

JPC_API const JPC_PathConstraintPath* JPC_PathConstraint_GetPath(const JPC_PathConstraint* self) {
	return to_jpc(to_jph(self)->GetPath());
}

JPC_API JPC_MotorSettings JPC_PathConstraint_GetPositionMotorSettings(const JPC_PathConstraint* self) {
	JPC_MotorSettings out;
	JPC_MotorSettings_to_jpc(&out, &to_jph(self)->GetPositionMotorSettings());
	return out;
}

JPC_API void JPC_PathConstraint_SetPositionMotorSettings(JPC_PathConstraint* self, const JPC_MotorSettings* inSettings) {
	JPC_MotorSettings_to_jph(inSettings, &to_jph(self)->GetPositionMotorSettings());
}

////////////////////////////////////////////////////////////////////////////////
// Vehicle wheel runtime

OPAQUE_WRAPPER(JPC_WheelWV, JPH::WheelWV)

JPC_API uint JPC_VehicleConstraint_GetWheelCount(const JPC_VehicleConstraint* self) {
	return static_cast<uint>(to_jph(self)->GetWheels().size());
}

JPC_API JPC_WheelWV* JPC_VehicleConstraint_GetWheel(JPC_VehicleConstraint* self, uint inIndex) {
	return to_jpc(static_cast<JPH::WheelWV*>(to_jph(self)->GetWheel(inIndex)));
}

JPC_API bool JPC_WheelWV_HasContact(const JPC_WheelWV* self) {
	return to_jph(self)->HasContact();
}

JPC_API float JPC_WheelWV_GetAngularVelocity(const JPC_WheelWV* self) {
	return to_jph(self)->GetAngularVelocity();
}

JPC_API void JPC_WheelWV_SetAngularVelocity(JPC_WheelWV* self, float inVel) {
	to_jph(self)->SetAngularVelocity(inVel);
}

JPC_API float JPC_WheelWV_GetRotationAngle(const JPC_WheelWV* self) {
	return to_jph(self)->GetRotationAngle();
}

JPC_API void JPC_WheelWV_SetRotationAngle(JPC_WheelWV* self, float inAngle) {
	to_jph(self)->SetRotationAngle(inAngle);
}

JPC_API float JPC_WheelWV_GetSteerAngle(const JPC_WheelWV* self) {
	return to_jph(self)->GetSteerAngle();
}

JPC_API void JPC_WheelWV_SetSteerAngle(JPC_WheelWV* self, float inAngle) {
	to_jph(self)->SetSteerAngle(inAngle);
}

JPC_API JPC_BodyID JPC_WheelWV_GetContactBodyID(const JPC_WheelWV* self) {
	return to_jpc(to_jph(self)->GetContactBodyID());
}

JPC_API JPC_RVec3 JPC_WheelWV_GetContactPosition(const JPC_WheelWV* self) {
	return to_jpc(to_jph(self)->GetContactPosition());
}

JPC_API JPC_Vec3 JPC_WheelWV_GetContactNormal(const JPC_WheelWV* self) {
	return to_jpc(to_jph(self)->GetContactNormal());
}

JPC_API JPC_Vec3 JPC_WheelWV_GetContactPointVelocity(const JPC_WheelWV* self) {
	return to_jpc(to_jph(self)->GetContactPointVelocity());
}

JPC_API JPC_Vec3 JPC_WheelWV_GetContactLongitudinal(const JPC_WheelWV* self) {
	return to_jpc(to_jph(self)->GetContactLongitudinal());
}

JPC_API JPC_Vec3 JPC_WheelWV_GetContactLateral(const JPC_WheelWV* self) {
	return to_jpc(to_jph(self)->GetContactLateral());
}

JPC_API float JPC_WheelWV_GetSuspensionLength(const JPC_WheelWV* self) {
	return to_jph(self)->GetSuspensionLength();
}

JPC_API float JPC_WheelWV_GetSuspensionLambda(const JPC_WheelWV* self) {
	return to_jph(self)->GetSuspensionLambda();
}

JPC_API float JPC_WheelWV_GetLongitudinalLambda(const JPC_WheelWV* self) {
	return to_jph(self)->GetLongitudinalLambda();
}

JPC_API float JPC_WheelWV_GetLateralLambda(const JPC_WheelWV* self) {
	return to_jph(self)->GetLateralLambda();
}

JPC_API void JPC_WheeledVehicleController_SetForwardInput(JPC_WheeledVehicleController* self, float inForward) {
	to_jph(self)->SetForwardInput(inForward);
}

JPC_API float JPC_WheeledVehicleController_GetForwardInput(const JPC_WheeledVehicleController* self) {
	return to_jph(self)->GetForwardInput();
}

JPC_API void JPC_WheeledVehicleController_SetRightInput(JPC_WheeledVehicleController* self, float inRight) {
	to_jph(self)->SetRightInput(inRight);
}

JPC_API float JPC_WheeledVehicleController_GetRightInput(const JPC_WheeledVehicleController* self) {
	return to_jph(self)->GetRightInput();
}

JPC_API void JPC_WheeledVehicleController_SetBrakeInput(JPC_WheeledVehicleController* self, float inBrake) {
	to_jph(self)->SetBrakeInput(inBrake);
}

JPC_API float JPC_WheeledVehicleController_GetBrakeInput(const JPC_WheeledVehicleController* self) {
	return to_jph(self)->GetBrakeInput();
}

JPC_API void JPC_WheeledVehicleController_SetHandBrakeInput(JPC_WheeledVehicleController* self, float inHandBrake) {
	to_jph(self)->SetHandBrakeInput(inHandBrake);
}

JPC_API float JPC_WheeledVehicleController_GetHandBrakeInput(const JPC_WheeledVehicleController* self) {
	return to_jph(self)->GetHandBrakeInput();
}

////////////////////////////////////////////////////////////////////////////////
// SoftBody additional constraint types

JPC_API void JPC_SoftBodySharedSettings_AddDihedralBend(JPC_SoftBodySharedSettings* self, const JPC_SoftBodyDihedralBend* inBend) {
	JPH::SoftBodySharedSettings::DihedralBend b;
	b.mVertex[0] = inBend->Vertex[0];
	b.mVertex[1] = inBend->Vertex[1];
	b.mVertex[2] = inBend->Vertex[2];
	b.mVertex[3] = inBend->Vertex[3];
	b.mCompliance = inBend->Compliance;
	b.mInitialAngle = inBend->InitialAngle;
	to_jph(self)->mDihedralBendConstraints.push_back(b);
}

JPC_API void JPC_SoftBodySharedSettings_AddVolumeConstraint(JPC_SoftBodySharedSettings* self, const JPC_SoftBodyVolume* inVolume) {
	JPH::SoftBodySharedSettings::Volume v;
	v.mVertex[0] = inVolume->Vertex[0];
	v.mVertex[1] = inVolume->Vertex[1];
	v.mVertex[2] = inVolume->Vertex[2];
	v.mVertex[3] = inVolume->Vertex[3];
	v.mSixRestVolume = inVolume->SixRestVolume;
	v.mCompliance = inVolume->Compliance;
	to_jph(self)->mVolumeConstraints.push_back(v);
}

JPC_API void JPC_SoftBodySharedSettings_AddSkinned(JPC_SoftBodySharedSettings* self, const JPC_SoftBodySkinned* inSkinned) {
	JPH::SoftBodySharedSettings::Skinned s;
	s.mVertex = inSkinned->Vertex;
	for (int i = 0; i < 4; i++) {
		s.mWeights[i].mInvBindIndex = inSkinned->Weights[i].InvBindIndex;
		s.mWeights[i].mWeight = inSkinned->Weights[i].Weight;
	}
	s.mMaxDistance = inSkinned->MaxDistance;
	s.mBackStopDistance = inSkinned->BackStopDistance;
	s.mBackStopRadius = inSkinned->BackStopRadius;
	to_jph(self)->mSkinnedConstraints.push_back(s);
}

JPC_API void JPC_SoftBodySharedSettings_AddInvBind(JPC_SoftBodySharedSettings* self, const JPC_SoftBodyInvBind* inInvBind) {
	JPH::SoftBodySharedSettings::InvBind b;
	b.mJointIndex = inInvBind->JointIndex;
	b.mInvBind = to_jph(inInvBind->InvBind);
	to_jph(self)->mInvBindMatrices.push_back(b);
}

JPC_API void JPC_SoftBodySharedSettings_AddLRA(JPC_SoftBodySharedSettings* self, const JPC_SoftBodyLRA* inLRA) {
	JPH::SoftBodySharedSettings::LRA l;
	l.mVertex[0] = inLRA->Vertex[0];
	l.mVertex[1] = inLRA->Vertex[1];
	l.mMaxDistance = inLRA->MaxDistance;
	to_jph(self)->mLRAConstraints.push_back(l);
}

JPC_API void JPC_SoftBodySharedSettings_CalculateEdgeLengths(JPC_SoftBodySharedSettings* self) {
	to_jph(self)->CalculateEdgeLengths();
}

JPC_API void JPC_SoftBodySharedSettings_CalculateBendConstraintConstants(JPC_SoftBodySharedSettings* self) {
	to_jph(self)->CalculateBendConstraintConstants();
}

JPC_API void JPC_SoftBodySharedSettings_CalculateVolumeConstraintVolumes(JPC_SoftBodySharedSettings* self) {
	to_jph(self)->CalculateVolumeConstraintVolumes();
}

JPC_API void JPC_SoftBodySharedSettings_CalculateLRALengths(JPC_SoftBodySharedSettings* self, float inMaxDistanceMultiplier) {
	to_jph(self)->CalculateLRALengths(inMaxDistanceMultiplier);
}

JPC_API void JPC_SoftBodySharedSettings_CalculateSkinnedConstraintNormals(JPC_SoftBodySharedSettings* self) {
	to_jph(self)->CalculateSkinnedConstraintNormals();
}

////////////////////////////////////////////////////////////////////////////////
// PhysicsMaterial default + custom bridge

JPC_API const JPC_PhysicsMaterial* JPC_PhysicsMaterial_GetDefault() {
	return to_jpc(JPH::PhysicsMaterial::sDefault.GetPtr());
}

class JPC_PhysicsMaterialBridge final : public JPH::PhysicsMaterial {
public:
	void* mUserData;
	JPC_PhysicsMaterialFns mFns;

	JPC_PhysicsMaterialBridge(void* inUserData, JPC_PhysicsMaterialFns inFns)
		: mUserData(inUserData), mFns(inFns) {}

	virtual const char* GetDebugName() const override {
		if (mFns.GetDebugName)
			return mFns.GetDebugName(mUserData);
		return "Custom";
	}

	virtual JPH::Color GetDebugColor() const override {
		if (mFns.GetDebugColor) {
			JPC_Color c = mFns.GetDebugColor(mUserData);
			return JPH::Color(c.r, c.g, c.b, c.a);
		}
		return JPH::Color::sGrey;
	}
};

JPC_API JPC_PhysicsMaterial* JPC_PhysicsMaterial_new(void* inUserData, JPC_PhysicsMaterialFns inFns) {
	auto* bridge = new JPC_PhysicsMaterialBridge(inUserData, inFns);
	bridge->AddRef();
	return to_jpc(static_cast<JPH::PhysicsMaterial*>(bridge));
}

////////////////////////////////////////////////////////////////////////////////
// ConstraintSettingsObj — opaque handle for constraint settings returned at runtime

OPAQUE_WRAPPER(JPC_ConstraintSettingsObj, JPH::ConstraintSettings)

JPC_API JPC_ConstraintSettingsObj* JPC_Constraint_GetConstraintSettings(const JPC_Constraint* self) {
	JPH::Ref<JPH::ConstraintSettings> ref = to_jph(self)->GetConstraintSettings();
	JPH::ConstraintSettings* raw = ref.GetPtr();
	if (raw)
		raw->AddRef();
	return to_jpc(raw);
}

JPC_API void JPC_ConstraintSettingsObj_AddRef(const JPC_ConstraintSettingsObj* self) {
	const_cast<JPH::ConstraintSettings*>(to_jph(self))->AddRef();
}

JPC_API void JPC_ConstraintSettingsObj_Release(const JPC_ConstraintSettingsObj* self) {
	const_cast<JPH::ConstraintSettings*>(to_jph(self))->Release();
}

JPC_API JPC_ConstraintSettings JPC_ConstraintSettingsObj_GetBaseSettings(const JPC_ConstraintSettingsObj* self) {
	JPC_ConstraintSettings out;
	JPC_ConstraintSettings_to_jpc(&out, to_jph(self));
	return out;
}

////////////////////////////////////////////////////////////////////////////////
// Ragdoll pose (matrix overloads)

JPC_API void JPC_Ragdoll_SetPose(JPC_Ragdoll* self, JPC_RVec3 inRootOffset, const JPC_Mat44* inJointMatrices, uint32_t inCount, bool inLockBodies) {
	to_jph(self)->SetPose(
		JPH::RVec3(inRootOffset.x, inRootOffset.y, inRootOffset.z),
		reinterpret_cast<const JPH::Mat44*>(inJointMatrices),
		inLockBodies);
	(void)inCount;
}

JPC_API void JPC_Ragdoll_GetPose(JPC_Ragdoll* self, JPC_RVec3* outRootOffset, JPC_Mat44* outJointMatrices, uint32_t inCount, bool inLockBodies) {
	JPH::RVec3 rootOffset;
	to_jph(self)->GetPose(rootOffset, reinterpret_cast<JPH::Mat44*>(outJointMatrices), inLockBodies);
	outRootOffset->x = rootOffset.GetX();
	outRootOffset->y = rootOffset.GetY();
	outRootOffset->z = rootOffset.GetZ();
	(void)inCount;
}

JPC_API void JPC_Ragdoll_DriveToPoseUsingKinematics(JPC_Ragdoll* self, JPC_RVec3 inRootOffset, const JPC_Mat44* inJointMatrices, uint32_t inCount, float inDeltaTime, bool inLockBodies) {
	to_jph(self)->DriveToPoseUsingKinematics(
		JPH::RVec3(inRootOffset.x, inRootOffset.y, inRootOffset.z),
		reinterpret_cast<const JPH::Mat44*>(inJointMatrices),
		inDeltaTime,
		inLockBodies);
	(void)inCount;
}

////////////////////////////////////////////////////////////////////////////////
// SoftBodySharedSettings — CreateConstraints + material assignment

JPC_API void JPC_SoftBodySharedSettings_CreateConstraints(
	JPC_SoftBodySharedSettings* self,
	const JPC_SoftBodyVertexAttributes* inVertexAttributes,
	uint32_t inVertexAttributesCount,
	JPC_SoftBodyEBendType inBendType,
	float inAngleTolerance)
{
	static_assert(sizeof(JPC_SoftBodyVertexAttributes) == sizeof(JPH::SoftBodySharedSettings::VertexAttributes));
	to_jph(self)->CreateConstraints(
		reinterpret_cast<const JPH::SoftBodySharedSettings::VertexAttributes*>(inVertexAttributes),
		inVertexAttributesCount,
		static_cast<JPH::SoftBodySharedSettings::EBendType>(inBendType),
		inAngleTolerance);
}

JPC_API void JPC_SoftBodySharedSettings_AddMaterial(JPC_SoftBodySharedSettings* self, const JPC_PhysicsMaterial* inMaterial) {
	to_jph(self)->mMaterials.push_back(const_cast<JPH::PhysicsMaterial*>(to_jph(inMaterial)));
}

////////////////////////////////////////////////////////////////////////////////
// VehicleConstraint wheel transform helpers

JPC_API void JPC_VehicleConstraint_GetWheelLocalBasis(const JPC_VehicleConstraint* self, uint32_t inWheelIndex, JPC_Vec3* outForward, JPC_Vec3* outUp, JPC_Vec3* outRight) {
	JPH::Vec3 forward, up, right;
	to_jph(self)->GetWheelLocalBasis(to_jph(self)->GetWheel(inWheelIndex), forward, up, right);
	*outForward = { forward.GetX(), forward.GetY(), forward.GetZ() };
	*outUp      = { up.GetX(),      up.GetY(),      up.GetZ()      };
	*outRight   = { right.GetX(),   right.GetY(),   right.GetZ()   };
}

JPC_API JPC_Mat44 JPC_VehicleConstraint_GetWheelLocalTransform(const JPC_VehicleConstraint* self, uint32_t inWheelIndex, JPC_Vec3 inWheelRight, JPC_Vec3 inWheelUp) {
	JPH::Mat44 result = to_jph(self)->GetWheelLocalTransform(
		inWheelIndex,
		JPH::Vec3(inWheelRight.x, inWheelRight.y, inWheelRight.z),
		JPH::Vec3(inWheelUp.x, inWheelUp.y, inWheelUp.z));
	JPC_Mat44 out;
	memcpy(&out, &result, sizeof(JPC_Mat44));
	return out;
}

JPC_API JPC_RMat44 JPC_VehicleConstraint_GetWheelWorldTransform(const JPC_VehicleConstraint* self, uint32_t inWheelIndex, JPC_Vec3 inWheelRight, JPC_Vec3 inWheelUp) {
	JPH::RMat44 result = to_jph(self)->GetWheelWorldTransform(
		inWheelIndex,
		JPH::Vec3(inWheelRight.x, inWheelRight.y, inWheelRight.z),
		JPH::Vec3(inWheelUp.x, inWheelUp.y, inWheelUp.z));
	JPC_RMat44 out;
	memcpy(&out, &result, sizeof(JPC_RMat44));
	return out;
}

////////////////////////////////////////////////////////////////////////////////
// PhysicsSettings get/set

JPC_API JPC_PhysicsSettings JPC_PhysicsSystem_GetPhysicsSettings(const JPC_PhysicsSystem* self) {
	const JPH::PhysicsSettings& s = to_jph(self)->GetPhysicsSettings();
	JPC_PhysicsSettings out;
	out.MaxInFlightBodyPairs             = s.mMaxInFlightBodyPairs;
	out.StepListenersBatchSize           = s.mStepListenersBatchSize;
	out.StepListenerBatchesPerJob        = s.mStepListenerBatchesPerJob;
	out.Baumgarte                        = s.mBaumgarte;
	out.SpeculativeContactDistance       = s.mSpeculativeContactDistance;
	out.PenetrationSlop                  = s.mPenetrationSlop;
	out.LinearCastThreshold              = s.mLinearCastThreshold;
	out.LinearCastMaxPenetration         = s.mLinearCastMaxPenetration;
	out.ManifoldTolerance                = s.mManifoldTolerance;
	out.MaxPenetrationDistance           = s.mMaxPenetrationDistance;
	out.BodyPairCacheMaxDeltaPositionSq  = s.mBodyPairCacheMaxDeltaPositionSq;
	out.BodyPairCacheCosMaxDeltaRotationDiv2 = s.mBodyPairCacheCosMaxDeltaRotationDiv2;
	out.ContactNormalCosMaxDeltaRotation = s.mContactNormalCosMaxDeltaRotation;
	out.ContactPointPreserveLambdaMaxDistSq = s.mContactPointPreserveLambdaMaxDistSq;
	out.NumVelocitySteps                 = s.mNumVelocitySteps;
	out.NumPositionSteps                 = s.mNumPositionSteps;
	out.MinVelocityForRestitution        = s.mMinVelocityForRestitution;
	out.TimeBeforeSleep                  = s.mTimeBeforeSleep;
	out.PointVelocitySleepThreshold      = s.mPointVelocitySleepThreshold;
	out.DeterministicSimulation          = s.mDeterministicSimulation;
	out.ConstraintWarmStart              = s.mConstraintWarmStart;
	out.UseBodyPairContactCache          = s.mUseBodyPairContactCache;
	out.UseManifoldReduction             = s.mUseManifoldReduction;
	out.UseLargeIslandSplitter           = s.mUseLargeIslandSplitter;
	out.AllowSleeping                    = s.mAllowSleeping;
	out.CheckActiveEdges                 = s.mCheckActiveEdges;
	return out;
}

JPC_API void JPC_PhysicsSystem_SetPhysicsSettings(JPC_PhysicsSystem* self, JPC_PhysicsSettings in) {
	JPH::PhysicsSettings s;
	s.mMaxInFlightBodyPairs             = in.MaxInFlightBodyPairs;
	s.mStepListenersBatchSize           = in.StepListenersBatchSize;
	s.mStepListenerBatchesPerJob        = in.StepListenerBatchesPerJob;
	s.mBaumgarte                        = in.Baumgarte;
	s.mSpeculativeContactDistance       = in.SpeculativeContactDistance;
	s.mPenetrationSlop                  = in.PenetrationSlop;
	s.mLinearCastThreshold              = in.LinearCastThreshold;
	s.mLinearCastMaxPenetration         = in.LinearCastMaxPenetration;
	s.mManifoldTolerance                = in.ManifoldTolerance;
	s.mMaxPenetrationDistance           = in.MaxPenetrationDistance;
	s.mBodyPairCacheMaxDeltaPositionSq  = in.BodyPairCacheMaxDeltaPositionSq;
	s.mBodyPairCacheCosMaxDeltaRotationDiv2 = in.BodyPairCacheCosMaxDeltaRotationDiv2;
	s.mContactNormalCosMaxDeltaRotation = in.ContactNormalCosMaxDeltaRotation;
	s.mContactPointPreserveLambdaMaxDistSq = in.ContactPointPreserveLambdaMaxDistSq;
	s.mNumVelocitySteps                 = in.NumVelocitySteps;
	s.mNumPositionSteps                 = in.NumPositionSteps;
	s.mMinVelocityForRestitution        = in.MinVelocityForRestitution;
	s.mTimeBeforeSleep                  = in.TimeBeforeSleep;
	s.mPointVelocitySleepThreshold      = in.PointVelocitySleepThreshold;
	s.mDeterministicSimulation          = in.DeterministicSimulation;
	s.mConstraintWarmStart              = in.ConstraintWarmStart;
	s.mUseBodyPairContactCache          = in.UseBodyPairContactCache;
	s.mUseManifoldReduction             = in.UseManifoldReduction;
	s.mUseLargeIslandSplitter           = in.UseLargeIslandSplitter;
	s.mAllowSleeping                    = in.AllowSleeping;
	s.mCheckActiveEdges                 = in.CheckActiveEdges;
	to_jph(self)->SetPhysicsSettings(s);
}

////////////////////////////////////////////////////////////////////////////////
// BodyStats getter

JPC_API JPC_BodyStats JPC_PhysicsSystem_GetBodyStats(const JPC_PhysicsSystem* self) {
	JPH::BodyManager::BodyStats s = to_jph(self)->GetBodyStats();
	JPC_BodyStats out;
	out.NumBodies               = s.mNumBodies;
	out.MaxBodies               = s.mMaxBodies;
	out.NumBodiesStatic         = s.mNumBodiesStatic;
	out.NumBodiesDynamic        = s.mNumBodiesDynamic;
	out.NumActiveBodiesDynamic  = s.mNumActiveBodiesDynamic;
	out.NumBodiesKinematic      = s.mNumBodiesKinematic;
	out.NumActiveBodiesKinematic = s.mNumActiveBodiesKinematic;
	out.NumSoftBodies           = s.mNumSoftBodies;
	out.NumActiveSoftBodies     = s.mNumActiveSoftBodies;
	return out;
}

////////////////////////////////////////////////////////////////////////////////
// PhysicsSystem batch constraint / no-lock interfaces / misc

JPC_API void JPC_PhysicsSystem_AddConstraints(JPC_PhysicsSystem* self, JPC_Constraint** inConstraints, int inNumber) {
	to_jph(self)->AddConstraints(reinterpret_cast<JPH::Constraint**>(inConstraints), inNumber);
}

JPC_API void JPC_PhysicsSystem_RemoveConstraints(JPC_PhysicsSystem* self, JPC_Constraint** inConstraints, int inNumber) {
	to_jph(self)->RemoveConstraints(reinterpret_cast<JPH::Constraint**>(inConstraints), inNumber);
}

JPC_API const JPC_BodyInterface* JPC_PhysicsSystem_GetBodyInterfaceNoLock(const JPC_PhysicsSystem* self) {
	return to_jpc(&to_jph(self)->GetBodyInterfaceNoLock());
}

JPC_API JPC_BodyInterface* JPC_PhysicsSystem_GetBodyInterfaceNoLockMut(JPC_PhysicsSystem* self) {
	return to_jpc(&to_jph(self)->GetBodyInterfaceNoLock());
}

JPC_API const JPC_NarrowPhaseQuery* JPC_PhysicsSystem_GetNarrowPhaseQueryNoLock(const JPC_PhysicsSystem* self) {
	return to_jpc(&to_jph(self)->GetNarrowPhaseQueryNoLock());
}

JPC_API const JPC_BodyLockInterface* JPC_PhysicsSystem_GetBodyLockInterfaceNoLock(const JPC_PhysicsSystem* self) {
	return to_jpc(&to_jph(self)->GetBodyLockInterfaceNoLock());
}

JPC_API bool JPC_PhysicsSystem_WereBodiesInContact(const JPC_PhysicsSystem* self, JPC_BodyID inBody1ID, JPC_BodyID inBody2ID) {
	return to_jph(self)->WereBodiesInContact(JPH::BodyID(inBody1ID), JPH::BodyID(inBody2ID));
}

JPC_API JPC_AABox JPC_PhysicsSystem_GetBounds(const JPC_PhysicsSystem* self) {
	JPH::AABox box = to_jph(self)->GetBounds();
	JPC_AABox out;
	memcpy(&out, &box, sizeof(JPC_AABox));
	return out;
}

JPC_API void JPC_PhysicsSystem_DrawConstraintLimits(JPC_PhysicsSystem* self, JPC_DebugRendererSimple* inRenderer) {
#ifdef JPH_DEBUG_RENDERER
	to_jph(self)->DrawConstraintLimits(to_jph(inRenderer));
#else
	(void)self; (void)inRenderer;
#endif
}

JPC_API void JPC_PhysicsSystem_DrawConstraintReferenceFrame(JPC_PhysicsSystem* self, JPC_DebugRendererSimple* inRenderer) {
#ifdef JPH_DEBUG_RENDERER
	to_jph(self)->DrawConstraintReferenceFrame(to_jph(inRenderer));
#else
	(void)self; (void)inRenderer;
#endif
}

////////////////////////////////////////////////////////////////////////////////
// BodyActivationListener vtable bridge

class JPC_BodyActivationListenerBridge;
OPAQUE_WRAPPER(JPC_BodyActivationListener, JPC_BodyActivationListenerBridge)

class JPC_BodyActivationListenerBridge final : public JPH::BodyActivationListener {
public:
	explicit JPC_BodyActivationListenerBridge(void* self, JPC_BodyActivationListenerFns fns) : self(self), fns(fns) {}

	void OnBodyActivated(const JPH::BodyID& inBodyID, JPH::uint64 inBodyUserData) override {
		if (fns.OnBodyActivated)
			fns.OnBodyActivated(self, inBodyID.GetIndexAndSequenceNumber(), inBodyUserData);
	}

	void OnBodyDeactivated(const JPH::BodyID& inBodyID, JPH::uint64 inBodyUserData) override {
		if (fns.OnBodyDeactivated)
			fns.OnBodyDeactivated(self, inBodyID.GetIndexAndSequenceNumber(), inBodyUserData);
	}

private:
	void* self;
	JPC_BodyActivationListenerFns fns;
};

DESTRUCTOR(JPC_BodyActivationListener)

JPC_API JPC_BodyActivationListener* JPC_BodyActivationListener_new(void* self, JPC_BodyActivationListenerFns fns) {
	return to_jpc(new JPC_BodyActivationListenerBridge(self, fns));
}

JPC_API void JPC_PhysicsSystem_SetBodyActivationListener(JPC_PhysicsSystem* self, JPC_BodyActivationListener* inListener) {
	to_jph(self)->SetBodyActivationListener(to_jph(inListener));
}

JPC_API JPC_BodyActivationListener* JPC_PhysicsSystem_GetBodyActivationListener(const JPC_PhysicsSystem* self) {
	return to_jpc(static_cast<JPC_BodyActivationListenerBridge*>(to_jph(self)->GetBodyActivationListener()));
}

////////////////////////////////////////////////////////////////////////////////
// CombineFunction
// Jolt requires a non-capturing plain function pointer. We store the user
// callback in globals and bridge through static trampoline functions.
// This works correctly for single-system usage and for multiple systems
// as long as they all use the same combine function (the common case).

static JPC_CombineFunction g_combine_friction_fn     = nullptr;
static JPC_CombineFunction g_combine_restitution_fn  = nullptr;

static float CombineFrictionBridge(const JPH::Body& b1, const JPH::SubShapeID& ss1, const JPH::Body& b2, const JPH::SubShapeID& ss2) {
	return g_combine_friction_fn(to_jpc(&b1), ss1.GetValue(), to_jpc(&b2), ss2.GetValue());
}

static float CombineRestitutionBridge(const JPH::Body& b1, const JPH::SubShapeID& ss1, const JPH::Body& b2, const JPH::SubShapeID& ss2) {
	return g_combine_restitution_fn(to_jpc(&b1), ss1.GetValue(), to_jpc(&b2), ss2.GetValue());
}

JPC_API void JPC_PhysicsSystem_SetCombineFriction(JPC_PhysicsSystem* self, JPC_CombineFunction inCombineFriction) {
	g_combine_friction_fn = inCombineFriction;
	to_jph(self)->SetCombineFriction(inCombineFriction ? CombineFrictionBridge : nullptr);
}

JPC_API void JPC_PhysicsSystem_SetCombineRestitution(JPC_PhysicsSystem* self, JPC_CombineFunction inCombineRestitution) {
	g_combine_restitution_fn = inCombineRestitution;
	to_jph(self)->SetCombineRestitution(inCombineRestitution ? CombineRestitutionBridge : nullptr);
}

////////////////////////////////////////////////////////////////////////////////
// SoftBodyContactListener vtable bridge

class JPC_SoftBodyContactListenerBridge;
OPAQUE_WRAPPER(JPC_SoftBodyContactListener, JPC_SoftBodyContactListenerBridge)
OPAQUE_WRAPPER(JPC_SoftBodyManifold, JPH::SoftBodyManifold)

class JPC_SoftBodyContactListenerBridge final : public JPH::SoftBodyContactListener {
public:
	explicit JPC_SoftBodyContactListenerBridge(void* self, JPC_SoftBodyContactListenerFns fns) : self(self), fns(fns) {}

	JPH::SoftBodyValidateResult OnSoftBodyContactValidate(const JPH::Body& inSoftBody, const JPH::Body& inOtherBody, JPH::SoftBodyContactSettings& ioSettings) override {
		if (!fns.OnSoftBodyContactValidate)
			return JPH::SoftBodyValidateResult::AcceptContact;
		JPC_SoftBodyContactSettings cs;
		cs.InvMassScale1   = ioSettings.mInvMassScale1;
		cs.InvMassScale2   = ioSettings.mInvMassScale2;
		cs.InvInertiaScale2 = ioSettings.mInvInertiaScale2;
		cs.IsSensor        = ioSettings.mIsSensor;
		uint32_t result = fns.OnSoftBodyContactValidate(self, to_jpc(&inSoftBody), to_jpc(&inOtherBody), &cs);
		ioSettings.mInvMassScale1   = cs.InvMassScale1;
		ioSettings.mInvMassScale2   = cs.InvMassScale2;
		ioSettings.mInvInertiaScale2 = cs.InvInertiaScale2;
		ioSettings.mIsSensor        = cs.IsSensor;
		return result == JPC_SOFT_BODY_VALIDATE_REJECT_CONTACT
			? JPH::SoftBodyValidateResult::RejectContact
			: JPH::SoftBodyValidateResult::AcceptContact;
	}

	void OnSoftBodyContactAdded(const JPH::Body& inSoftBody, const JPH::SoftBodyManifold& inManifold) override {
		if (fns.OnSoftBodyContactAdded)
			fns.OnSoftBodyContactAdded(self, to_jpc(&inSoftBody), to_jpc(&inManifold));
	}

private:
	void* self;
	JPC_SoftBodyContactListenerFns fns;
};

DESTRUCTOR(JPC_SoftBodyContactListener)

JPC_API JPC_SoftBodyContactListener* JPC_SoftBodyContactListener_new(void* self, JPC_SoftBodyContactListenerFns fns) {
	return to_jpc(new JPC_SoftBodyContactListenerBridge(self, fns));
}

JPC_API void JPC_PhysicsSystem_SetSoftBodyContactListener(JPC_PhysicsSystem* self, JPC_SoftBodyContactListener* inListener) {
	to_jph(self)->SetSoftBodyContactListener(to_jph(inListener));
}

JPC_API JPC_SoftBodyContactListener* JPC_PhysicsSystem_GetSoftBodyContactListener(const JPC_PhysicsSystem* self) {
	return to_jpc(static_cast<JPC_SoftBodyContactListenerBridge*>(to_jph(self)->GetSoftBodyContactListener()));
}

// SoftBodyManifold accessors
JPC_API JPC_BodyID JPC_SoftBodyManifold_GetBodyID(const JPC_SoftBodyManifold* self) {
	// return the body ID of the first vertex that has a contact, or invalid if none
	const JPH::SoftBodyManifold& m = *to_jph(self);
	for (const JPH::SoftBodyVertex& v : m.GetVertices()) {
		if (m.HasContact(v))
			return m.GetContactBodyID(v).GetIndexAndSequenceNumber();
	}
	return JPH::BodyID().GetIndexAndSequenceNumber();
}

JPC_API uint32_t JPC_SoftBodyManifold_GetNumVertices(const JPC_SoftBodyManifold* self) {
	return static_cast<uint32_t>(to_jph(self)->GetVertices().size());
}

JPC_API bool JPC_SoftBodyManifold_VertexHasContact(const JPC_SoftBodyManifold* self, uint32_t inVertexIndex) {
	const auto& verts = to_jph(self)->GetVertices();
	if (inVertexIndex >= verts.size())
		return false;
	return to_jph(self)->HasContact(verts[inVertexIndex]);
}

////////////////////////////////////////////////////////////////////////////////
// BodyInterface missing methods

JPC_API bool JPC_BodyInterface_IsSensor(const JPC_BodyInterface* self, JPC_BodyID inBodyID) {
	return to_jph(self)->IsSensor(JPH::BodyID(inBodyID));
}

JPC_API void JPC_BodyInterface_SetIsSensor(JPC_BodyInterface* self, JPC_BodyID inBodyID, bool inIsSensor) {
	to_jph(self)->SetIsSensor(JPH::BodyID(inBodyID), inIsSensor);
}

JPC_API float JPC_BodyInterface_GetMaxLinearVelocity(const JPC_BodyInterface* self, JPC_BodyID inBodyID) {
	return to_jph(self)->GetMaxLinearVelocity(JPH::BodyID(inBodyID));
}

JPC_API void JPC_BodyInterface_SetMaxLinearVelocity(JPC_BodyInterface* self, JPC_BodyID inBodyID, float inMaxLinearVelocity) {
	to_jph(self)->SetMaxLinearVelocity(JPH::BodyID(inBodyID), inMaxLinearVelocity);
}

JPC_API float JPC_BodyInterface_GetMaxAngularVelocity(const JPC_BodyInterface* self, JPC_BodyID inBodyID) {
	return to_jph(self)->GetMaxAngularVelocity(JPH::BodyID(inBodyID));
}

JPC_API void JPC_BodyInterface_SetMaxAngularVelocity(JPC_BodyInterface* self, JPC_BodyID inBodyID, float inMaxAngularVelocity) {
	to_jph(self)->SetMaxAngularVelocity(JPH::BodyID(inBodyID), inMaxAngularVelocity);
}

JPC_API void JPC_BodyInterface_ResetSleepTimer(JPC_BodyInterface* self, JPC_BodyID inBodyID) {
	to_jph(self)->ResetSleepTimer(JPH::BodyID(inBodyID));
}

JPC_API JPC_CollisionGroup JPC_BodyInterface_GetCollisionGroup(const JPC_BodyInterface* self, JPC_BodyID inBodyID) {
	JPH::CollisionGroup g = to_jph(self)->GetCollisionGroup(JPH::BodyID(inBodyID));
	JPC_CollisionGroup out;
	static_assert(sizeof(JPC_CollisionGroup) == sizeof(JPH::CollisionGroup));
	memcpy(&out, &g, sizeof(out));
	return out;
}

JPC_API void JPC_BodyInterface_SetCollisionGroup(JPC_BodyInterface* self, JPC_BodyID inBodyID, const JPC_CollisionGroup* inGroup) {
	JPH::CollisionGroup g;
	memcpy(&g, inGroup, sizeof(g));
	to_jph(self)->SetCollisionGroup(JPH::BodyID(inBodyID), g);
}

////////////////////////////////////////////////////////////////////////////////
// Character missing methods

JPC_API void JPC_Character_AddImpulse(JPC_Character* self, JPC_Vec3 inImpulse) {
	to_jph(self)->AddImpulse(JPH::Vec3(inImpulse.x, inImpulse.y, inImpulse.z));
}

JPC_API void JPC_Character_GetPositionAndRotation(const JPC_Character* self, JPC_RVec3* outPosition, JPC_Quat* outRotation, bool inLockBodies) {
	JPH::RVec3 pos;
	JPH::Quat  rot;
	to_jph(self)->GetPositionAndRotation(pos, rot, inLockBodies);
	*outPosition = { pos.GetX(), pos.GetY(), pos.GetZ() };
	*outRotation = { rot.GetX(), rot.GetY(), rot.GetZ(), rot.GetW() };
}

JPC_API void JPC_Character_SetLinearAndAngularVelocity(JPC_Character* self, JPC_Vec3 inLinearVelocity, JPC_Vec3 inAngularVelocity, bool inLockBodies) {
	to_jph(self)->SetLinearAndAngularVelocity(
		JPH::Vec3(inLinearVelocity.x,  inLinearVelocity.y,  inLinearVelocity.z),
		JPH::Vec3(inAngularVelocity.x, inAngularVelocity.y, inAngularVelocity.z),
		inLockBodies);
}

JPC_API JPC_RMat44 JPC_Character_GetWorldTransform(const JPC_Character* self, bool inLockBodies) {
	JPH::RMat44 m = to_jph(self)->GetWorldTransform(inLockBodies);
	JPC_RMat44 out;
	memcpy(&out, &m, sizeof(JPC_RMat44));
	return out;
}

////////////////////////////////////////////////////////////////////////////////
// Ragdoll missing methods

JPC_API void JPC_Ragdoll_SetLinearAndAngularVelocity(JPC_Ragdoll* self, JPC_Vec3 inLinearVelocity, JPC_Vec3 inAngularVelocity, bool inLockBodies) {
	to_jph(self)->SetLinearAndAngularVelocity(
		JPH::Vec3(inLinearVelocity.x,  inLinearVelocity.y,  inLinearVelocity.z),
		JPH::Vec3(inAngularVelocity.x, inAngularVelocity.y, inAngularVelocity.z),
		inLockBodies);
}

JPC_API JPC_AABox JPC_Ragdoll_GetWorldSpaceBounds(const JPC_Ragdoll* self, bool inLockBodies) {
	JPH::AABox box = to_jph(self)->GetWorldSpaceBounds(inLockBodies);
	JPC_AABox out;
	memcpy(&out, &box, sizeof(JPC_AABox));
	return out;
}

JPC_API void JPC_Ragdoll_GetRootTransform(const JPC_Ragdoll* self, JPC_RVec3* outPosition, JPC_Quat* outRotation, bool inLockBodies) {
	JPH::RVec3 pos;
	JPH::Quat  rot;
	to_jph(self)->GetRootTransform(pos, rot, inLockBodies);
	*outPosition = { pos.GetX(), pos.GetY(), pos.GetZ() };
	*outRotation = { rot.GetX(), rot.GetY(), rot.GetZ(), rot.GetW() };
}

////////////////////////////////////////////////////////////////////////////////
// VehicleConstraint additional methods

JPC_API JPC_Body* JPC_VehicleConstraint_GetVehicleBody(const JPC_VehicleConstraint* self) {
	return to_jpc(to_jph(self)->GetVehicleBody());
}

JPC_API float JPC_VehicleConstraint_GetMaxPitchRollAngle(const JPC_VehicleConstraint* self) {
	return to_jph(self)->GetMaxPitchRollAngle();
}

JPC_API void JPC_VehicleConstraint_SetMaxPitchRollAngle(JPC_VehicleConstraint* self, float inMaxPitchRollAngle) {
	to_jph(self)->SetMaxPitchRollAngle(inMaxPitchRollAngle);
}

JPC_API uint JPC_VehicleConstraint_GetNumStepsBetweenCollisionTestActive(const JPC_VehicleConstraint* self) {
	return to_jph(self)->GetNumStepsBetweenCollisionTestActive();
}

JPC_API void JPC_VehicleConstraint_SetNumStepsBetweenCollisionTestActive(JPC_VehicleConstraint* self, uint inNumSteps) {
	to_jph(self)->SetNumStepsBetweenCollisionTestActive(inNumSteps);
}

JPC_API uint JPC_VehicleConstraint_GetNumStepsBetweenCollisionTestInactive(const JPC_VehicleConstraint* self) {
	return to_jph(self)->GetNumStepsBetweenCollisionTestInactive();
}

JPC_API void JPC_VehicleConstraint_SetNumStepsBetweenCollisionTestInactive(JPC_VehicleConstraint* self, uint inNumSteps) {
	to_jph(self)->SetNumStepsBetweenCollisionTestInactive(inNumSteps);
}

////////////////////////////////////////////////////////////////////////////////
// BroadPhaseQuery

#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseQuery.h>
#include <Jolt/Physics/Collision/AABoxCast.h>
#include <Jolt/Geometry/OrientedBox.h>

OPAQUE_WRAPPER(JPC_BroadPhaseQuery, JPH::BroadPhaseQuery)

// --- BodyIDCollector bridge ---

class JPC_BodyIDCollectorBridge;
OPAQUE_WRAPPER(JPC_BodyIDCollector, JPC_BodyIDCollectorBridge)

class JPC_BodyIDCollectorBridge final : public JPH::CollisionCollector<JPH::BodyID, JPH::CollisionCollectorTraitsCollideShape> {
public:
	explicit JPC_BodyIDCollectorBridge(void* self, JPC_BodyIDCollectorFns fns) : self(self), fns(fns) {}

	void Reset() override {
		JPH::CollisionCollector<JPH::BodyID, JPH::CollisionCollectorTraitsCollideShape>::Reset();
		if (fns.Reset) fns.Reset(self);
	}

	void AddHit(const JPH::BodyID& bodyId) override {
		if (fns.AddHit) fns.AddHit(self, bodyId.GetIndexAndSequenceNumber());
	}

private:
	void* self;
	JPC_BodyIDCollectorFns fns;
};

DESTRUCTOR(JPC_BodyIDCollector)

JPC_API JPC_BodyIDCollector* JPC_BodyIDCollector_new(void* self, JPC_BodyIDCollectorFns fns) {
	return to_jpc(new JPC_BodyIDCollectorBridge(self, fns));
}

// --- BroadPhaseCastCollector bridge ---

class JPC_BroadPhaseCastCollectorBridge;
OPAQUE_WRAPPER(JPC_BroadPhaseCastCollector, JPC_BroadPhaseCastCollectorBridge)

class JPC_BroadPhaseCastCollectorBridge final : public JPH::CollisionCollector<JPH::BroadPhaseCastResult, JPH::CollisionCollectorTraitsCastRay> {
public:
	explicit JPC_BroadPhaseCastCollectorBridge(void* self, JPC_BroadPhaseCastCollectorFns fns) : self(self), fns(fns) {}

	void Reset() override {
		JPH::CollisionCollector<JPH::BroadPhaseCastResult, JPH::CollisionCollectorTraitsCastRay>::Reset();
		if (fns.Reset) fns.Reset(self);
	}

	void AddHit(const JPH::BroadPhaseCastResult& result) override {
		JPC_BroadPhaseCastResult r;
		r.BodyID   = result.mBodyID.GetIndexAndSequenceNumber();
		r.Fraction = result.mFraction;
		JPC_BroadPhaseCastCollector* base = to_jpc(this);
		if (fns.AddHit) fns.AddHit(self, base, &r);
	}

private:
	void* self;
	JPC_BroadPhaseCastCollectorFns fns;
};

DESTRUCTOR(JPC_BroadPhaseCastCollector)

JPC_API JPC_BroadPhaseCastCollector* JPC_BroadPhaseCastCollector_new(void* self, JPC_BroadPhaseCastCollectorFns fns) {
	return to_jpc(new JPC_BroadPhaseCastCollectorBridge(self, fns));
}

JPC_API void JPC_BroadPhaseCastCollector_UpdateEarlyOutFraction(JPC_BroadPhaseCastCollector* self, float inFraction) {
	to_jph(self)->UpdateEarlyOutFraction(inFraction);
}

// --- BroadPhaseQuery methods ---

JPC_API const JPC_BroadPhaseQuery* JPC_PhysicsSystem_GetBroadPhaseQuery(const JPC_PhysicsSystem* self) {
	return to_jpc(&to_jph(self)->GetBroadPhaseQuery());
}

// helper to get a const reference to a filter from nullable pointer
static const JPH::BroadPhaseLayerFilter& bpl_filter_ref(const JPC_BroadPhaseLayerFilter* p) {
	static const JPH::BroadPhaseLayerFilter sDefault{};
	return p ? *to_jph(p) : sDefault;
}

static const JPH::ObjectLayerFilter& ol_filter_ref(const JPC_ObjectLayerFilter* p) {
	static const JPH::ObjectLayerFilter sDefault{};
	return p ? *to_jph(p) : sDefault;
}

JPC_API void JPC_BroadPhaseQuery_CastRay(const JPC_BroadPhaseQuery* self, JPC_Vec3 inOrigin, JPC_Vec3 inDirection, JPC_BroadPhaseCastCollector* ioCollector, const JPC_BroadPhaseLayerFilter* inBPLayerFilter, const JPC_ObjectLayerFilter* inOLFilter) {
	to_jph(self)->CastRay(
		JPH::RayCast{ JPH::Vec3(inOrigin.x, inOrigin.y, inOrigin.z), JPH::Vec3(inDirection.x, inDirection.y, inDirection.z) },
		*to_jph(ioCollector),
		bpl_filter_ref(inBPLayerFilter),
		ol_filter_ref(inOLFilter));
}

JPC_API void JPC_BroadPhaseQuery_CollideAABox(const JPC_BroadPhaseQuery* self, const JPC_AABox* inBox, JPC_BodyIDCollector* ioCollector, const JPC_BroadPhaseLayerFilter* inBPLayerFilter, const JPC_ObjectLayerFilter* inOLFilter) {
	JPH::AABox box;
	memcpy(&box, inBox, sizeof(JPH::AABox));
	to_jph(self)->CollideAABox(box, *to_jph(ioCollector), bpl_filter_ref(inBPLayerFilter), ol_filter_ref(inOLFilter));
}

JPC_API void JPC_BroadPhaseQuery_CollideSphere(const JPC_BroadPhaseQuery* self, JPC_Vec3 inCenter, float inRadius, JPC_BodyIDCollector* ioCollector, const JPC_BroadPhaseLayerFilter* inBPLayerFilter, const JPC_ObjectLayerFilter* inOLFilter) {
	to_jph(self)->CollideSphere(
		JPH::Vec3(inCenter.x, inCenter.y, inCenter.z), inRadius,
		*to_jph(ioCollector),
		bpl_filter_ref(inBPLayerFilter),
		ol_filter_ref(inOLFilter));
}

JPC_API void JPC_BroadPhaseQuery_CollidePoint(const JPC_BroadPhaseQuery* self, JPC_Vec3 inPoint, JPC_BodyIDCollector* ioCollector, const JPC_BroadPhaseLayerFilter* inBPLayerFilter, const JPC_ObjectLayerFilter* inOLFilter) {
	to_jph(self)->CollidePoint(
		JPH::Vec3(inPoint.x, inPoint.y, inPoint.z),
		*to_jph(ioCollector),
		bpl_filter_ref(inBPLayerFilter),
		ol_filter_ref(inOLFilter));
}

JPC_API void JPC_BroadPhaseQuery_CollideOrientedBox(const JPC_BroadPhaseQuery* self, const JPC_OrientedBox* inBox, JPC_BodyIDCollector* ioCollector, const JPC_BroadPhaseLayerFilter* inBPLayerFilter, const JPC_ObjectLayerFilter* inOLFilter) {
	JPH::Mat44 orient;
	memcpy(&orient, &inBox->Orientation, sizeof(JPH::Mat44));
	JPH::OrientedBox box(orient, JPH::Vec3(inBox->HalfExtents.x, inBox->HalfExtents.y, inBox->HalfExtents.z));
	to_jph(self)->CollideOrientedBox(box, *to_jph(ioCollector), bpl_filter_ref(inBPLayerFilter), ol_filter_ref(inOLFilter));
}

// CastAABox needs CastShapeBodyCollector (TraitsCastShape), not TraitsCastRay.
// Adapt by wrapping the user's bridge in a local TraitsCastShape collector.
class JPC_BroadPhaseCastAABoxCollectorAdapter : public JPH::CastShapeBodyCollector {
public:
	explicit JPC_BroadPhaseCastAABoxCollectorAdapter(JPC_BroadPhaseCastCollectorBridge& wrapped)
		: mWrapped(wrapped) {}

	void Reset() override {
		JPH::CastShapeBodyCollector::Reset();
		mWrapped.Reset();
	}

	void AddHit(const JPH::BroadPhaseCastResult& result) override {
		mWrapped.AddHit(result);
	}

private:
	JPC_BroadPhaseCastCollectorBridge& mWrapped;
};

JPC_API void JPC_BroadPhaseQuery_CastAABox(const JPC_BroadPhaseQuery* self, const JPC_AABoxCast* inBox, JPC_BroadPhaseCastCollector* ioCollector, const JPC_BroadPhaseLayerFilter* inBPLayerFilter, const JPC_ObjectLayerFilter* inOLFilter) {
	JPH::AABox box;
	memcpy(&box, &inBox->Box, sizeof(JPH::AABox));
	JPH::AABoxCast cast{ box, JPH::Vec3(inBox->Direction.x, inBox->Direction.y, inBox->Direction.z) };
	JPC_BroadPhaseCastAABoxCollectorAdapter adapter(*to_jph(ioCollector));
	to_jph(self)->CastAABox(cast, adapter, bpl_filter_ref(inBPLayerFilter), ol_filter_ref(inOLFilter));
}

JPC_API JPC_AABox JPC_BroadPhaseQuery_GetBounds(const JPC_BroadPhaseQuery* self) {
	JPH::AABox box = to_jph(self)->GetBounds();
	JPC_AABox out;
	memcpy(&out, &box, sizeof(JPC_AABox));
	return out;
}

////////////////////////////////////////////////////////////////////////////////
// MotionProperties

static inline JPH::MotionProperties* to_jph(JPC_MotionProperties* self) {
	return reinterpret_cast<JPH::MotionProperties*>(self);
}
static inline const JPH::MotionProperties* to_jph(const JPC_MotionProperties* self) {
	return reinterpret_cast<const JPH::MotionProperties*>(self);
}

JPC_API JPC_MotionQuality JPC_MotionProperties_GetMotionQuality(const JPC_MotionProperties* self) {
	return (JPC_MotionQuality)to_jph(self)->GetMotionQuality();
}

JPC_API JPC_AllowedDOFs JPC_MotionProperties_GetAllowedDOFs(const JPC_MotionProperties* self) {
	return (JPC_AllowedDOFs)to_jph(self)->GetAllowedDOFs();
}

JPC_API JPC_Vec3 JPC_MotionProperties_GetLinearVelocity(const JPC_MotionProperties* self) {
	return to_jpc(to_jph(self)->GetLinearVelocity());
}

JPC_API void JPC_MotionProperties_SetLinearVelocity(JPC_MotionProperties* self, JPC_Vec3 inVelocity) {
	to_jph(self)->SetLinearVelocity(to_jph(inVelocity));
}

JPC_API void JPC_MotionProperties_SetLinearVelocityClamped(JPC_MotionProperties* self, JPC_Vec3 inVelocity) {
	to_jph(self)->SetLinearVelocityClamped(to_jph(inVelocity));
}

JPC_API JPC_Vec3 JPC_MotionProperties_GetAngularVelocity(const JPC_MotionProperties* self) {
	return to_jpc(to_jph(self)->GetAngularVelocity());
}

JPC_API void JPC_MotionProperties_SetAngularVelocity(JPC_MotionProperties* self, JPC_Vec3 inVelocity) {
	to_jph(self)->SetAngularVelocity(to_jph(inVelocity));
}

JPC_API void JPC_MotionProperties_SetAngularVelocityClamped(JPC_MotionProperties* self, JPC_Vec3 inVelocity) {
	to_jph(self)->SetAngularVelocityClamped(to_jph(inVelocity));
}

JPC_API float JPC_MotionProperties_GetMaxLinearVelocity(const JPC_MotionProperties* self) {
	return to_jph(self)->GetMaxLinearVelocity();
}

JPC_API void JPC_MotionProperties_SetMaxLinearVelocity(JPC_MotionProperties* self, float inVelocity) {
	to_jph(self)->SetMaxLinearVelocity(inVelocity);
}

JPC_API float JPC_MotionProperties_GetMaxAngularVelocity(const JPC_MotionProperties* self) {
	return to_jph(self)->GetMaxAngularVelocity();
}

JPC_API void JPC_MotionProperties_SetMaxAngularVelocity(JPC_MotionProperties* self, float inVelocity) {
	to_jph(self)->SetMaxAngularVelocity(inVelocity);
}

JPC_API float JPC_MotionProperties_GetLinearDamping(const JPC_MotionProperties* self) {
	return to_jph(self)->GetLinearDamping();
}

JPC_API void JPC_MotionProperties_SetLinearDamping(JPC_MotionProperties* self, float inDamping) {
	to_jph(self)->SetLinearDamping(inDamping);
}

JPC_API float JPC_MotionProperties_GetAngularDamping(const JPC_MotionProperties* self) {
	return to_jph(self)->GetAngularDamping();
}

JPC_API void JPC_MotionProperties_SetAngularDamping(JPC_MotionProperties* self, float inDamping) {
	to_jph(self)->SetAngularDamping(inDamping);
}

JPC_API float JPC_MotionProperties_GetGravityFactor(const JPC_MotionProperties* self) {
	return to_jph(self)->GetGravityFactor();
}

JPC_API void JPC_MotionProperties_SetGravityFactor(JPC_MotionProperties* self, float inFactor) {
	to_jph(self)->SetGravityFactor(inFactor);
}

JPC_API float JPC_MotionProperties_GetInverseMass(const JPC_MotionProperties* self) {
	return to_jph(self)->GetInverseMass();
}

JPC_API float JPC_MotionProperties_GetInverseMassUnchecked(const JPC_MotionProperties* self) {
	return to_jph(self)->GetInverseMassUnchecked();
}

JPC_API void JPC_MotionProperties_SetInverseMass(JPC_MotionProperties* self, float inInverseMass) {
	to_jph(self)->SetInverseMass(inInverseMass);
}

////////////////////////////////////////////////////////////////////////////////
// TransformedShape

static inline JPH::TransformedShape* to_jph(JPC_TransformedShape* self) {
	return reinterpret_cast<JPH::TransformedShape*>(self);
}
static inline const JPH::TransformedShape* to_jph(const JPC_TransformedShape* self) {
	return reinterpret_cast<const JPH::TransformedShape*>(self);
}

JPC_API void JPC_TransformedShape_delete(JPC_TransformedShape* self) {
	delete to_jph(self);
}

JPC_API JPC_BodyID JPC_TransformedShape_GetBodyID(const JPC_TransformedShape* self) {
	return to_jpc(to_jph(self)->mBodyID);
}

JPC_API const JPC_Shape* JPC_TransformedShape_GetShape(const JPC_TransformedShape* self) {
	return to_jpc(to_jph(self)->mShape.GetPtr());
}

JPC_API JPC_Vec3 JPC_TransformedShape_GetShapeScale(const JPC_TransformedShape* self) {
	return to_jpc(to_jph(self)->GetShapeScale());
}

JPC_API JPC_RMat44 JPC_TransformedShape_GetCenterOfMassTransform(const JPC_TransformedShape* self) {
	return to_jpc(to_jph(self)->GetCenterOfMassTransform());
}

JPC_API JPC_RMat44 JPC_TransformedShape_GetInverseCenterOfMassTransform(const JPC_TransformedShape* self) {
	return to_jpc(to_jph(self)->GetInverseCenterOfMassTransform());
}

JPC_API JPC_RMat44 JPC_TransformedShape_GetWorldTransform(const JPC_TransformedShape* self) {
	return to_jpc(to_jph(self)->GetWorldTransform());
}

JPC_API JPC_AABox JPC_TransformedShape_GetWorldSpaceBounds(const JPC_TransformedShape* self) {
	JPH::AABox box = to_jph(self)->GetWorldSpaceBounds();
	JPC_AABox out;
	out.Min = to_jpc(box.mMin);
	out.Max = to_jpc(box.mMax);
	return out;
}
