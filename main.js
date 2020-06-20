import {Vector} from './types.js';
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

const universe = [];
const planet = new Planet(30);
universe.push(planet);
const box = new Crate(new Vector(0.5, 0.5));
box.position = new Vector(0, -planet.radius - 0.25);
universe.push(box);
for (let i = 0; i < 5; i++) {
  const crate = new Crate(new Vector(0.5, 0.5));
  crate.position = box.position.sub(new Vector(0, 0.5 * (1 + i)));
  universe.push(crate);
}
window.box = box;
io.camera.position = box.position;

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
    const target = box.position;
    const current = io.camera.position;
    const offset = box.position.sub(io.camera.position);
    const offsetHalfLife = 0.5;
    const factor = 1 - Math.pow(0.5, deltaTime / offsetHalfLife);
    io.camera.position = current.add(offset.mul(factor));
    io.camera.angle = Math.atan2(box.gravity.y, box.gravity.x) - Math.PI / 2;
    io.draw(context => {
      for (const x of universe) x.draw(context);
    });
    await next;
  }
}

main();
