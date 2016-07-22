use std::collections::HashMap;
use std::fmt::Debug;
use serde::{Serialize, Serializer, Deserialize, Deserializer};

#[doc(hidden)] pub use b2::*;
use user_data::{UserDataTypes, UserData};

#[derive(Clone, Copy, Hash, Eq, PartialEq, Debug, Serialize, Deserialize)]
pub struct BodyId(pub usize);
#[derive(Clone, Copy, Hash, Eq, PartialEq, Debug, Serialize, Deserialize)]
pub struct JointId(pub usize);

pub struct IdToHandle {
    bodies: HashMap<BodyId, BodyHandle>,
    joints: HashMap<JointId, JointHandle>,
}

impl IdToHandle {
    pub fn new() -> Self {
        IdToHandle {
            bodies: HashMap::new(),
            joints: HashMap::new(),
        }
    }

    pub fn clear(&mut self) {
        self.bodies.clear();
        self.joints.clear();
    }

    pub fn body_handle(&self, id: BodyId) -> Option<BodyHandle> {
        self.bodies.get(&id).cloned()
    }

    pub fn insert_body(&mut self, id: BodyId, handle: BodyHandle) {
        if self.bodies.insert(id, handle).is_some() {
            panic!("body id duplicate");
        }
    }

    pub fn joint_handle(&self, id: JointId) -> Option<JointHandle> {
        self.joints.get(&id).cloned()
    }

    pub fn insert_joint(&mut self, id: JointId, handle: JointHandle) {
        if self.joints.insert(id, handle).is_some() {
            panic!("joint id duplicate");
        }
    }
}

// TODO: avoid this struct
#[derive(Serialize, Deserialize, Debug)]
pub struct WorldSnapshot<U: UserDataTypes>
    where U::BodyData: Debug,
          U::FixtureData: Debug,
          U::JointData: Debug,
{
    config: WorldConfigSnapshot,
    bodies: Vec<CompleteBodySnapshot<U>>,
    joints: Vec<CompleteJointSnapshot<U>>,
}

type CompleteBodySnapshot<U: UserDataTypes> = (BodyId, BodySnapshot, U::BodyData, Vec<CompleteFixtureSnapshot<U>>);
type CompleteFixtureSnapshot<U: UserDataTypes> = (FixtureSnapshot, U::FixtureData);
type CompleteJointSnapshot<U: UserDataTypes> = (JointId, JointSnapshot, U::JointData);

impl<U: UserDataTypes> WorldSnapshot<U>
    where U::BodyData: Debug,
          U::FixtureData: Debug,
          U::JointData: Debug,
{
    pub fn take(world: &World<U>) -> Self
        where U::BodyData: Serialize + Clone,
              U::FixtureData: Serialize + Clone,
              U::JointData: Serialize + Clone
    {
        let body_snapshots: Vec<_> = world.bodies()
            .map(|(_, body)| {
                let body: &MetaBody<U> = &body.borrow();
                let fixture_snapshots: Vec<_> = body.fixtures()
                    .map(|(_, fixture)| {
                        let fixture: &MetaFixture<U> = &fixture.borrow();
                        (FixtureSnapshot::take(fixture), fixture.user_data().clone())
                    })
                    .collect();

                let (id, s) = BodySnapshot::take(body);
                (id, s, body.user_data().clone(), fixture_snapshots)
            })
            .collect();

        let joint_snapshots: Vec<_> = world.joints()
            .map(|(_, joint)| {
                let joint: &MetaJoint<U> = &joint.borrow();
                let (id, s) = JointSnapshot::take(joint);
                (id, s, joint.user_data().clone())
            })
            .collect();

        WorldSnapshot {
            config: WorldConfigSnapshot::take(world),
            bodies: body_snapshots,
            joints: joint_snapshots,
        }
    }

    pub fn rebuild(&self, id_to_handle: &mut IdToHandle) -> World<U>
        where U::BodyData: Deserialize + Clone,
              U::FixtureData: Deserialize + Clone,
              U::JointData: Deserialize + Clone,
    {
        id_to_handle.clear();
        let mut world = self.config.rebuild();

        for &(id, ref snapshot, ref data, ref fixtures) in &self.bodies {
            let handle = snapshot.rebuild(&mut world, data.clone());
            id_to_handle.insert_body(id, handle);

            let mut body = world.body_mut(handle);
            for &(ref snapshot, ref data) in fixtures {
                snapshot.rebuild(&mut body, data.clone());
            }

            snapshot.may_restore_mass_data(&mut body);
        }

        let joints = self.joints.iter();
        let mut gear_joint_snapshots = Vec::new();
        for &(id, ref snapshot, ref data) in joints {
            match snapshot.rebuild(&mut world, data.clone(), id_to_handle) {
                Ok(handle) => id_to_handle.insert_joint(id, handle),
                Err(gjs) => gear_joint_snapshots.push((id, gjs, data)), 
            }
        }

        for (id, gjs, data) in gear_joint_snapshots {
            let handle = gjs.rebuild(&mut world, data.clone(), id_to_handle);
            id_to_handle.insert_joint(id, handle);
        }

        world
    }
}

macro_rules! snapshot {
    ($module:ident => $name:ident {
        $(pub $field_name:ident: $field_type:ty $([$default_path:expr => $default:expr])*,)*
     }) => {
        pub use self::$module::Snapshot as $name;
        #[doc(hidden)]
        pub mod $module {
            pub use super::*;
            
            #[derive(Serialize, Deserialize, Clone, Debug)]
            pub struct Snapshot {
                $(
                    $(#[serde(default=$default_path)])*
                    pub $field_name: $field_type
                ),*
            }
            
            pub mod default {
                pub use super::*;
                $($(pub fn $field_name() -> $field_type { $default })*)*
            }
        }
    }
}

snapshot! {
    world => WorldConfigSnapshot {
        pub gravity: [f32; 2],
        pub allow_sleep: bool ["default::allow_sleep" => true],
        pub auto_clear_forces: bool ["default::auto_clear_forces" => false],
        pub warm_starting: bool ["default::warm_starting" => true],
        pub continuous_physics: bool ["default::continuous_physics" => true],
        pub sub_stepping: bool ["default::sub_stepping" => false],
    }
}

impl WorldConfigSnapshot {
    pub fn take<U: UserDataTypes>(world: &World<U>) -> Self {
        WorldConfigSnapshot {
            gravity: world.gravity().into(),
            allow_sleep: world.is_sleeping_allowed(),
            auto_clear_forces: world.is_auto_clearing_forces(),
            warm_starting: world.is_warm_starting(),
            continuous_physics: world.is_continuous_physics(),
            sub_stepping: world.is_sub_stepping(),
        }
    }

    pub fn rebuild<U: UserDataTypes>(&self) -> World<U> {
        let mut world = World::new(&self.gravity.into());
        world.set_sleeping_allowed(self.allow_sleep);
        world.set_auto_clearing_forces(self.auto_clear_forces);
        world.set_warm_starting(self.warm_starting);
        world.set_continuous_physics(self.continuous_physics);
        world.set_sub_stepping(self.sub_stepping);
        world
    }
}

snapshot! {
    body => BodySnapshot {
        pub body_type: BodyType,
        pub position: [f32; 2],
        pub angle: f32 ["default::angle" => 0.],
        pub linear_velocity: [f32; 2] ["default::linear_velocity" => [0., 0.]],
        pub angular_velocity: f32 ["default::angular_velocity" => 0.],
        pub linear_damping: f32 ["default::linear_damping" => 0.],
        pub angular_damping: f32 ["default::angular_damping" => 0.],
        pub allow_sleep: bool ["default::allow_sleep" => true],
        pub awake: bool ["default::awake" => true],
        pub fixed_rotation: bool ["default::fixed_rotation" => false],
        pub bullet: bool ["default::bullet" => false],
        pub active: bool ["default::active" => true],
        pub gravity_scale: f32 ["default::gravity_scale" => 1.],
        pub mass_data: Option<MassSnapshot> ["default::mass_data" => None],
    }
}

impl BodySnapshot {
    pub fn take<U: UserDataTypes>(body: &MetaBody<U>) -> (BodyId, Self) {
        let snapshot = BodySnapshot {
            body_type: body.body_type(),
            position: (*body.position()).into(),
            angle: body.angle(),
            linear_velocity: (*body.linear_velocity()).into(),
            angular_velocity: body.angular_velocity(),
            linear_damping: body.linear_damping(),
            angular_damping: body.angular_damping(),
            allow_sleep: body.is_sleeping_allowed(),
            awake: body.is_awake(),
            fixed_rotation: body.is_rotation_fixed(),
            bullet: body.is_bullet(),
            active: body.is_active(),
            gravity_scale: body.gravity_scale(),
            mass_data: Some(MassSnapshot::take(&body.mass_data())),
        };

        (BodyId(body.handle().index()), snapshot)
    }

    pub fn rebuild<U: UserDataTypes>(&self, world: &mut World<U>, data: U::BodyData) -> BodyHandle {
        let def = BodyDef {
            body_type: self.body_type,
            position: self.position.into(),
            angle: self.angle,
            linear_velocity: self.linear_velocity.into(),
            angular_velocity: self.angular_velocity,
            linear_damping: self.linear_damping,
            angular_damping: self.angular_damping,
            allow_sleep: self.allow_sleep,
            awake: self.awake,
            fixed_rotation: self.fixed_rotation,
            bullet: self.bullet,
            active: self.active,
            gravity_scale: self.gravity_scale,
            .. BodyDef::new()
        };

        world.create_body_with(&def, data)
    }

    pub fn may_restore_mass_data<U: UserDataTypes>(&self, body: &mut MetaBody<U>) {
        self.mass_data.as_ref().map(|m| body.set_mass_data(&m.rebuild()));
    }
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct MassSnapshot {
    pub mass: f32,
    pub center: [f32; 2],
    pub inertia: f32,
}

impl MassSnapshot {
    pub fn take(data: &MassData) -> Self {
        MassSnapshot {
            mass: data.mass,
            center: data.center.into(),
            inertia: data.inertia,
        }
    }

    pub fn rebuild(&self) -> MassData {
        MassData {
            mass: self.mass,
            center: self.center.into(),
            inertia: self.inertia,
        }
    }
}

snapshot! {
    fixture => FixtureSnapshot {
        pub shape: ShapeSnapshot,
        pub friction: f32 ["default::friction" => 0.2],
        pub restitution: f32 ["default::restitution" => 0.],
        pub density: f32 ["default::density" => 0.],
        pub is_sensor: bool ["default::is_sensor" => false],
        pub filter: Filter ["default::filter" => Filter::new()],
    }
}

impl FixtureSnapshot {
    pub fn take<U: UserDataTypes>(fixture: &MetaFixture<U>) -> Self {
        FixtureSnapshot {
            shape: ShapeSnapshot::take(&fixture.shape()),
            friction: fixture.friction(),
            restitution: fixture.restitution(),
            density: fixture.density(),
            is_sensor: fixture.is_sensor(),
            filter: fixture.filter_data().clone(),
        }
    }

    pub fn rebuild<U: UserDataTypes>(&self, body: &mut MetaBody<U>, data: U::FixtureData) -> FixtureHandle {
        let shape: UnknownShape = self.shape.rebuild();
        let mut def = FixtureDef {
            friction: self.friction,
            restitution: self.restitution,
            density: self.density,
            is_sensor: self.is_sensor,
            filter: self.filter.clone(),
            .. FixtureDef::new()
        };

        body.create_fixture_with(&shape, &mut def, data)
    }
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub enum ShapeSnapshot {
    Circle(CircleShapeSnapshot),
    Edge(EdgeShapeSnapshot),
    Polygon(PolygonShapeSnapshot),
    Chain(ChainShapeSnapshot),
}

impl ShapeSnapshot {
    pub fn take(shape: &UnknownShape) -> Self {
        use self::ShapeSnapshot::*;
        match shape {
            &UnknownShape::Unknown => panic!("truly unknown shape"),
            &UnknownShape::Circle(ref s) => Circle(CircleShapeSnapshot::take(s)),
            &UnknownShape::Edge(ref s) => Edge(EdgeShapeSnapshot::take(s)),
            &UnknownShape::Polygon(ref s) => Polygon(PolygonShapeSnapshot::take(s)),
            &UnknownShape::Chain(ref s) => Chain(ChainShapeSnapshot::take(s)),
        }
    }

    pub fn rebuild(&self) -> UnknownShape {
        use self::ShapeSnapshot::*;
        match self {
            &Circle(ref ss) => UnknownShape::Circle(ss.rebuild()),
            &Edge(ref ss) => UnknownShape::Edge(ss.rebuild()),
            &Polygon(ref ss) => UnknownShape::Polygon(ss.rebuild()),
            &Chain(ref ss) => UnknownShape::Chain(ss.rebuild()),
        }
    }
}

snapshot! {
    circle => CircleShapeSnapshot {
        pub position: [f32; 2],
        pub radius: f32,
    }
}

impl CircleShapeSnapshot {
    pub fn take(shape: &CircleShape) -> Self {
        CircleShapeSnapshot {
            position: shape.position().into(),
            radius: shape.radius(),
        }
    }

    pub fn rebuild(&self) -> CircleShape {
        CircleShape::new_with(self.position.into(), self.radius)
    }
}

snapshot! {
    // TODO
    edge => EdgeShapeSnapshot {}
}

impl EdgeShapeSnapshot {
    pub fn take(shape: &EdgeShape) -> Self {
        unimplemented!()
    }

    pub fn rebuild(&self) -> EdgeShape {
        unimplemented!()
    }
}

// TODO: avoid this Vec
snapshot! {
    polygon => PolygonShapeSnapshot {
        pub vertices: Vec<[f32; 2]>,
    }
}

impl PolygonShapeSnapshot {
    pub fn take(shape: &PolygonShape) -> Self {
        PolygonShapeSnapshot {
            vertices: (0..shape.vertex_count())
                .map(|i| (*shape.vertex(i)).into())
                .collect()
        }
    }

    pub fn rebuild(&self) -> PolygonShape {
        let vertices: Vec<_> = self.vertices.iter().map(|&v| v.into()).collect();
        PolygonShape::new_with(&vertices)
    }
}

snapshot! {
    // TODO
    chain => ChainShapeSnapshot {}
}

impl ChainShapeSnapshot {
    pub fn take(shape: &ChainShape) -> Self {
        unimplemented!()
    }

    pub fn rebuild(&self) -> ChainShape {
        unimplemented!()
    }
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub enum JointSnapshot {
    Revolute(RevoluteJointSnapshot),
    Prismatic(PrismaticJointSnapshot),
    Distance(DistanceJointSnapshot),
    Pulley(PulleyJointSnapshot),
    Mouse(MouseJointSnapshot),
    Gear(GearJointSnapshot),
    Wheel(WheelJointSnapshot),
    Weld(WeldJointSnapshot),
    Friction(FrictionJointSnapshot),
    Rope(RopeJointSnapshot),
    Motor(MotorJointSnapshot),
}

impl JointSnapshot {
    pub fn take<U: UserDataTypes>(joint: &MetaJoint<U>) -> (JointId, Self) {
        use self::JointSnapshot::*;
        let snapshot = match joint as &UnknownJoint {
            &UnknownJoint::Unknown => panic!("truly unknown joint"),
            &UnknownJoint::Revolute(ref j) => Revolute(RevoluteJointSnapshot::take(j)),
            &UnknownJoint::Prismatic(ref j) => Prismatic(PrismaticJointSnapshot::take(j)),
            &UnknownJoint::Distance(ref j) => Distance(DistanceJointSnapshot::take(j)),
            &UnknownJoint::Pulley(ref j) => Pulley(PulleyJointSnapshot::take(j)),
            &UnknownJoint::Mouse(ref j) => Mouse(MouseJointSnapshot::take(j)),
            &UnknownJoint::Gear(ref j) => Gear(GearJointSnapshot::take(j)),
            &UnknownJoint::Wheel(ref j) => Wheel(WheelJointSnapshot::take(j)),
            &UnknownJoint::Weld(ref j) => Weld(WeldJointSnapshot::take(j)),
            &UnknownJoint::Friction(ref j) => Friction(FrictionJointSnapshot::take(j)),
            &UnknownJoint::Rope(ref j) => Rope(RopeJointSnapshot::take(j)),
            &UnknownJoint::Motor(ref j) => Motor(MotorJointSnapshot::take(j)),
        };

        (JointId(joint.handle().index()), snapshot)
    }

    pub fn rebuild<'a, U: UserDataTypes>(&'a self,
                                         world: &mut World<U>,
                                         data: U::JointData,
                                         id_to_handle: &mut IdToHandle)
                                         -> Result<JointHandle, &'a GearJointSnapshot>
    {
        use self::JointSnapshot::*;
        let value = match self {
            &Revolute(ref js) => js.rebuild(world, data, id_to_handle),
            &Prismatic(ref js) => js.rebuild(world, data, id_to_handle),
            &Distance(ref js) => js.rebuild(world, data, id_to_handle),
            &Pulley(ref js) => js.rebuild(world, data, id_to_handle),
            &Mouse(ref js) => js.rebuild(world, data, id_to_handle),
            &Gear(ref js) => return Err(js),
            &Wheel(ref js) => js.rebuild(world, data, id_to_handle),
            &Weld(ref js) => js.rebuild(world, data, id_to_handle),
            &Friction(ref js) => js.rebuild(world, data, id_to_handle),
            &Rope(ref js) => js.rebuild(world, data, id_to_handle),
            &Motor(ref js) => js.rebuild(world, data, id_to_handle),
        };

        Ok(value)
    }
}

snapshot! {
    revolute => RevoluteJointSnapshot {
        pub body_a: BodyId,
        pub body_b: BodyId,
        pub collide_connected: bool ["default::collide_connected" => false],
        pub local_anchor_a: [f32; 2] ["default::local_anchor_a" => [0., 0.]],
        pub local_anchor_b: [f32; 2] ["default::local_anchor_b" => [0., 0.]],
        pub reference_angle: f32 ["default::reference_angle" => 0.],
        pub enable_limit: bool ["default::enable_limit" => false],
        pub lower_angle: f32 ["default::lower_angle" => 0.],
        pub upper_angle: f32 ["default::upper_angle" => 0.],
        pub enable_motor: bool ["default::enable_motor" => false],
        pub motor_speed: f32 ["default::motor_speed" => 0.],
        pub max_motor_torque: f32 ["default::max_motor_torque" => 0.],
    }
}

impl RevoluteJointSnapshot {
    pub fn take(joint: &RevoluteJoint) -> Self {
        unimplemented!()
    }

    pub fn rebuild<U: UserDataTypes>(&self,
                                     world: &mut World<U>,
                                     data: U::JointData,
                                     id_to_handle: &mut IdToHandle)
                                     -> JointHandle
    {
        unimplemented!()
    }
}

snapshot! {
    // TODO
    prismatic => PrismaticJointSnapshot {}
}

impl PrismaticJointSnapshot {
    pub fn take(joint: &PrismaticJoint) -> Self {
        unimplemented!()
    }

    pub fn rebuild<U: UserDataTypes>(&self,
                                     world: &mut World<U>,
                                     data: U::JointData,
                                     id_to_handle: &mut IdToHandle)
                                     -> JointHandle
    {
        unimplemented!()
    }
}

snapshot! {
    // TODO
    distance => DistanceJointSnapshot {}
}

impl DistanceJointSnapshot {
    pub fn take(joint: &DistanceJoint) -> Self {
        unimplemented!()
    }

    pub fn rebuild<U: UserDataTypes>(&self,
                                     world: &mut World<U>,
                                     data: U::JointData,
                                     id_to_handle: &mut IdToHandle)
                                     -> JointHandle
    {
        unimplemented!()
    }
}

snapshot! {
    // TODO
    pulley => PulleyJointSnapshot {}
}

impl PulleyJointSnapshot {
    pub fn take(joint: &PulleyJoint) -> Self {
        unimplemented!()
    }

    pub fn rebuild<U: UserDataTypes>(&self,
                                     world: &mut World<U>,
                                     data: U::JointData,
                                     id_to_handle: &mut IdToHandle)
                                     -> JointHandle
    {
        unimplemented!()
    }
}

snapshot! {
    // TODO
    mouse => MouseJointSnapshot {}
}

impl MouseJointSnapshot {
    pub fn take(joint: &MouseJoint) -> Self {
        unimplemented!()
    }

    pub fn rebuild<U: UserDataTypes>(&self,
                                     world: &mut World<U>,
                                     data: U::JointData,
                                     id_to_handle: &mut IdToHandle)
                                     -> JointHandle
    {
        unimplemented!()
    }
}

snapshot! {
    gear => GearJointSnapshot {
        pub collide_connected: bool ["default::collide_connected" => false],
        pub joint_1: JointId,
        pub joint_2: JointId,
        pub ratio: f32 ["default::ratio" => 1.],
    }
}

impl GearJointSnapshot {
    pub fn take(joint: &GearJoint) -> Self {
        GearJointSnapshot {
            collide_connected: joint.is_collide_connected(),
            joint_1: JointId(joint.joint_1().index()),
            joint_2: JointId(joint.joint_2().index()),
            ratio: joint.ratio(),
        }
    }

    pub fn rebuild<U: UserDataTypes>(&self,
                                     world: &mut World<U>,
                                     data: U::JointData,
                                     id_to_handle: &mut IdToHandle)
                                     -> JointHandle
    {
        let joint_1 = id_to_handle.joint_handle(self.joint_1)
            .unwrap_or_else(|| panic!("no handle for this joint id"));
        let joint_2 = id_to_handle.joint_handle(self.joint_2)
            .unwrap_or_else(|| panic!("no handle for this joint id"));

        let def = GearJointDef {
            collide_connected: self.collide_connected,
            joint_1: joint_1,
            joint_2: joint_2,
            ratio: self.ratio,
        };

        world.create_joint_with(&def, data)
    }
}

snapshot! {
    // TODO
    wheel => WheelJointSnapshot {}
}

impl WheelJointSnapshot {
    pub fn take(joint: &WheelJoint) -> Self {
        unimplemented!()
    }

    pub fn rebuild<U: UserDataTypes>(&self,
                                     world: &mut World<U>,
                                     data: U::JointData,
                                     id_to_handle: &mut IdToHandle)
                                     -> JointHandle
    {
        unimplemented!()
    }
}

snapshot! {
    // TODO
    weld => WeldJointSnapshot {}
}

impl WeldJointSnapshot {
    pub fn take(joint: &WeldJoint) -> Self {
        unimplemented!()
    }

    pub fn rebuild<U: UserDataTypes>(&self,
                                     world: &mut World<U>,
                                     data: U::JointData,
                                     id_to_handle: &mut IdToHandle)
                                     -> JointHandle
    {
        unimplemented!()
    }
}

snapshot! {
    // TODO
    friction => FrictionJointSnapshot {}
}

impl FrictionJointSnapshot {
    pub fn take(joint: &FrictionJoint) -> Self {
        unimplemented!()
    }

    pub fn rebuild<U: UserDataTypes>(&self,
                                     world: &mut World<U>,
                                     data: U::JointData,
                                     id_to_handle: &mut IdToHandle)
                                     -> JointHandle
    {
        unimplemented!()
    }
}

snapshot! {
    // TODO
    rope => RopeJointSnapshot {}
}

impl RopeJointSnapshot {
    pub fn take(joint: &RopeJoint) -> Self {
        unimplemented!()
    }

    pub fn rebuild<U: UserDataTypes>(&self,
                                     world: &mut World<U>,
                                     data: U::JointData,
                                     id_to_handle: &mut IdToHandle)
                                     -> JointHandle
    {
        unimplemented!()
    }
}

snapshot! {
    // TODO
    motor => MotorJointSnapshot {}
}

impl MotorJointSnapshot {
    pub fn take(joint: &MotorJoint) -> Self {
        unimplemented!()
    }

    pub fn rebuild<U: UserDataTypes>(&self,
                                     world: &mut World<U>,
                                     data: U::JointData,
                                     id_to_handle: &mut IdToHandle)
                                     -> JointHandle
    {
        unimplemented!()
    }
}