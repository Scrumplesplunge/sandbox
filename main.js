import {Vector, Matrix} from './types.js';
import * as io from './io.js';
import {PhysicsObject, AABB, Circle, intersect} from './physics.js';

class Planet extends PhysicsObject {
  constructor(radius) {
    super(new Circle(new Vector, radius, 128));
    this.radius = radius;
    this.inverseMass = 1e-15;
    this.inverseInertia = 1e-20;
  }
  draw(context) {
    context.save();
    context.translate(this.position.x, this.position.y);
    context.rotate(this.angle);
    const r = 2 * this.radius;
    context.drawImage(Planet.image, -r, -r, 2 * r, 2 * r);
    context.restore();
  }
}
Planet.image = new Image;
Planet.image.src = 'planet.png';

class Crate extends PhysicsObject {
  constructor(size) {
    super(new AABB(size.mul(-0.5), size.mul(0.5)));
    this.size = size;
    this.restitution = 0.2;
  }
  draw(context) {
    context.save();
      context.translate(this.position.x, this.position.y);
      context.rotate(this.angle);
      const extent = this.size.mul(0.5);
      context.drawImage(
          Crate.image, -extent.x, -extent.y, this.size.x, this.size.y);
    context.restore();
  }
}
Crate.image = new Image;
Crate.image.src = "crate.png";

const keys = new Set;
addEventListener('keydown', event => keys.add(event.code));
addEventListener('keyup', event => keys.delete(event.code));

class Thruster extends PhysicsObject {
  constructor(key, force, size) {
    super(new AABB(size.mul(-0.5), size.mul(0.5)));
    this.size = size;
    this.force = force;
    this.key = key;
    this.throttle = 0;
  }
  update(dt) {
    this.throttle = keys.has(this.key) ? 1 : 0;
    this.applyImpulse(
        Vector.fromAngle(this.angle)
            .rotate90()
            .mul(-dt * this.force * this.throttle),
        this.position);
    super.update(dt);
  }
  draw(context) {
    context.save();
      context.translate(this.position.x, this.position.y);
      context.rotate(this.angle);
      const extent = this.size.mul(0.5);
      const flameHeight = this.throttle * 2 * this.size.y;
      context.drawImage(
          Thruster.flameImage, -extent.x, extent.y, this.size.x, flameHeight);
      context.drawImage(
          Thruster.image, -extent.x, -extent.y, this.size.x, this.size.y);
    context.restore();
  }
}
Thruster.image = new Image;
Thruster.image.src = 'thruster.png';
Thruster.flameImage = new Image;
Thruster.flameImage.src = 'flame.png';

class Child {
  constructor(object, offset, angleOffset) {
    if (object.inverseMass == 0 || object.inverseInertia == 0) {
      throw new Error('Infinitely massive objects cannot be child nodes.');
    }
    this.object = object;
    this.offset = offset;
    this.angleOffset = angleOffset;
  }
}

class Group {
  constructor(children) {
    this.children = children;
    // Compute combined momentum.
    this.mass = 0;
    let offset = new Vector;
    for (const child of children) {
      const childMass = 1 / child.object.inverseMass;
      this.mass += childMass;
      offset = offset.add(child.offset.mul(childMass));
    }
    offset = offset.mul(1 / this.mass);
    // Ensure that the offsets are all relative to the centre of mass.
    for (const child of this.children) {
      child.offset = child.offset.sub(offset);
    }
    this.inertia = 0;
    for (const child of children) {
      const childMass = 1 / child.object.inverseMass;
      const childInertia = 1 / child.object.inverseInertia;
      this.inertia += childInertia + childMass * child.offset.dot(child.offset);
    }
  }
  update() {
    // Compute combined momentum.
    let momentum = new Vector;
    let massCenter = new Vector;
    let angularMomentum = 0;
    for (const child of this.children) {
      const childMass = 1 / child.object.inverseMass;
      const childInertia = 1 / child.object.inverseInertia;
      massCenter = massCenter.add(child.object.position.mul(childMass));
      momentum = momentum.add(child.object.velocity.mul(childMass));
      angularMomentum += child.object.angularVelocity * childInertia;
    }
    massCenter = massCenter.mul(1 / this.mass);
    // Add the angular momentum given by the linear component of children.
    for (const child of this.children) {
      const childMass = 1 / child.object.inverseMass;
      const r = child.object.position.sub(massCenter);
      // We need to account for objects being further apart than they are
      // supposed to be to avoid that introducing unwanted additional energy.
      const distance = r.length();
      const intendedDistance = child.offset.length();
      angularMomentum += childMass * r.cross(child.object.velocity) *
          intendedDistance / distance;
    }
    // Derive the expected velocities.
    const velocity = momentum.mul(1 / this.mass);
    const angularVelocity = angularMomentum / this.inertia;
    // Update velocities.
    for (const child of this.children) {
      const offset = child.object.position.sub(massCenter);
      const distance = offset.length();
      const intendedDistance = child.offset.length();
      const rotationalVelocity =
          offset.rotate90().mul(angularVelocity * intendedDistance / distance);
      child.object.angularVelocity = angularVelocity;
      child.object.velocity = velocity.add(rotationalVelocity);
    }
    // Correct the relative offsets of objects.
    let orientation = new Vector;
    for (const child of this.children) {
      const offset = child.object.position.sub(massCenter);
      const childMass = 1 / child.object.inverseMass;
      orientation = orientation.add(
          Vector.fromAngle(offset.angle() - child.offset.angle())
              .mul(childMass));
    }
    const angle = orientation.angle();
    const matrix = Matrix.rotate(angle);
    const amount = 0.8;
    for (const child of this.children) {
      const current = child.object.position;
      const target = massCenter.add(matrix.apply(child.offset));
      child.object.position = current.add(target.sub(current).mul(amount));
      child.object.angle = angle + child.angleOffset;
    }
  }
}

function weld(...objects) {
  return new Group(objects.map(o => new Child(o, o.position, o.angle)));
}

const universe = [];
const planet = new Planet(30);
universe.push(planet);
const origin = new Vector(0, -planet.radius - 0.25);
for (let y = 0; y < 5; y++) {
  for (let x = 0; x < 5; x++) {
    const crate = new Crate(new Vector(0.5, 0.5));
    crate.position = origin.sub(new Vector(0.5 * (x - 2), 0.5 * y));
    universe.push(crate);
  }
}
const a = new Thruster('KeyW', 40, new Vector(0.2, 0.2));
const b = new Thruster('KeyW', 40, new Vector(0.2, 0.2));
const c = new Crate(new Vector(1, 1));
const d = new Thruster('KeyA', 10, new Vector(0.1, 0.1));
const e = new Thruster('KeyD', 10, new Vector(0.1, 0.1));
a.position = origin.sub(new Vector(2.3, 1.4));
b.position = origin.sub(new Vector(2.7, 1.4));
c.position = origin.sub(new Vector(2.5, 2));
c.inverseMass = 0.2;
d.position = origin.sub(new Vector(1.95, 2.4));
d.angle = -Math.PI / 2;
d.inverseMass = 10;
d.inverseInertia = 10;
e.position = origin.sub(new Vector(3.05, 2.4));
e.angle = Math.PI / 2;
e.inverseMass = 10;
e.inverseInertia = 10;
universe.push(a, b, c, d, e);
const triangle = weld(a, b, c, d, e);
let focus = c;
io.camera.position = focus.position;

// Set the planet spinning.
planet.angularVelocity = 0.1;
// Create a transient weld for all items. This will make everything stationary
// relative to everything else.
weld(...universe).update();

let heldItem = null;
let holdOffset = new Vector;

io.mouseHandlers.down = () => {
  const mouse = io.cameraMouse();
  heldItem = null;
  for (const x of universe) {
    if (x.movable() && x.getMesh().contains(mouse)) {
      console.log("Holding " + x.constructor.name);
      heldItem = x;
      holdOffset = heldItem.toLocal(mouse);
      break;
    }
  }
};

io.mouseHandlers.up = () => heldItem = null;

const gravityFactor = 1e-14 * (planet.radius ** 2);
function applyGravity(dt) {
  universe.sort((a, b) => a.inverseMass - b.inverseMass);
  const n = universe.length;
  for (const x of universe) x.gravity = new Vector;
  let start = 0;
  while (start < n && universe[start].inverseMass == 0) start++;
  for (let a = start; a < n; a++) {
    const aMass = 1 / universe[a].inverseMass;
    // Anything weighing less than 1e10 has negligible effect on anything else.
    if (aMass < 1e10) continue;
    for (let b = a + 1; b < n; b++) {
      const bMass = 1 / universe[b].inverseMass;
      const offset = universe[b].position.sub(universe[a].position);
      const distanceSquared = offset.dot(offset);
      // GMm/r^2, but offset has length r so we need offset.mul(dt * GMm/r^3).
      const impulse = offset.mul(
          dt * gravityFactor * aMass * bMass *
          Math.pow(distanceSquared, -3 / 2));
      universe[a].gravity = universe[a].gravity.add(impulse);
      universe[b].gravity = universe[b].gravity.sub(impulse);
    }
  }
  for (const x of universe) x.applyImpulse(x.gravity, x.position);
}

function innerTick(dt) {
  applyGravity(dt);
  // Update objects.
  for (const x of universe) {
    x.update(dt);
  }
  triangle.update();
  // Resolve collisions.
  for (let i = 0, n = universe.length; i < n; i++) {
    const movableA = universe[i].movable();
    for (let j = i + 1; j < n; j++) {
      if (!movableA && !universe[j].movable()) continue;
      const collision = intersect(universe[i], universe[j]);
      if (!collision) continue;
      collision.resolve();
      collision.correct();
    }
  }
}

function delay(s) {
  return new Promise(resolve => setTimeout(resolve, 1000 * s));
}

async function main() {
  const deltaTime = 0.02;
  while (true) {
    const next = delay(deltaTime);
    // Apply forces to the held item.
    if (heldItem) {
      const target = io.cameraMouse();
      const current = heldItem.toWorld(holdOffset);
      const targetVelocity = target.sub(current).mul(1 / deltaTime);
      const currentVelocity = heldItem.velocity;
      const acceleration = targetVelocity.sub(currentVelocity);
      const impulse = acceleration.mul(0.5 / heldItem.inverseMass);
      heldItem.applyImpulse(impulse, current);
    }
    const rounds = 5;
    for (let i = 0; i < rounds; i++) innerTick(deltaTime / rounds);
    const target = focus.position;
    const current = io.camera.position;
    const offset = focus.position.sub(io.camera.position);
    const offsetHalfLife = 0.5;
    const factor = 1 - Math.pow(0.5, deltaTime / offsetHalfLife);
    io.camera.position = current.add(offset.mul(factor));
    io.camera.angle =
        Math.atan2(focus.gravity.y, focus.gravity.x) - Math.PI / 2;
    io.draw(context => {
      for (const x of universe) x.draw(context);
    });
    await next;
  }
}

main();
