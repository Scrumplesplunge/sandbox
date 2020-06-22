import {Vector, Matrix} from './types.js';

export class PhysicsObject {
  constructor(mesh = new Circle) {
    this.mesh = mesh;
    this.position = new Vector;
    this.velocity = new Vector;
    this.angle = 0;
    this.angularVelocity = 0;
    this.inverseMass = 1;
    this.inverseInertia = 4;
    this.restitution = 0.9;
    this.staticFriction = 1;
    this.dynamicFriction = 0.75;
  }
  getMesh() { return this.mesh.at(this.position, this.angle); }
  movable() {
    return this.inverseMass != 0 && this.inverseInertia != 0;
  }
  toLocal(v) {
    return Matrix.rotate(-this.angle).apply(v.sub(this.position));
  }
  toWorld(v) {
    return Matrix.rotate(this.angle).apply(v).add(this.position);
  }
  velocityAt(contact) {
    const offset = contact.sub(this.position);
    return this.velocity.add(offset.rotate90().mul(this.angularVelocity));
  }
  applyImpulse(impulse, contact) {
    this.velocity = this.velocity.add(impulse.mul(this.inverseMass));
    const offset = contact.sub(this.position);
    this.angularVelocity += offset.cross(impulse) * this.inverseInertia;
  }
  update(dt) {
    this.position = this.position.add(this.velocity.mul(dt));
    this.angle += this.angularVelocity * dt;
    const tau = 2 * Math.PI;
    this.angle = (this.angle % tau + tau) % tau;
  }
  draw(context) { this.getMesh().draw(context); }
}

export class Collision {
  constructor(a, b, contact, normal, depth, face = null) {
    this.a = a;
    this.b = b;
    this.contact = contact;
    this.normal = normal;
    this.depth = depth;
    this.face = face;
  }
  // Resolve a collision by applying impulses according to restitution.
  resolve() {
    const relativeVelocity =
        this.b.velocityAt(this.contact).sub(this.a.velocityAt(this.contact));
    const alongNormal = this.normal.dot(relativeVelocity);
    // Objects are separating already, so we don't need to apply any correction.
    if (alongNormal > 0) return;
    const e = this.a.restitution * this.b.restitution;
    const normalInertiaA =
        Math.pow(this.contact.sub(this.a.position).cross(this.normal), 2) *
        this.a.inverseInertia;
    const normalInertiaB =
        Math.pow(this.contact.sub(this.b.position).cross(this.normal), 2) *
        this.b.inverseInertia;
    const massFactor = 1 /
        (this.a.inverseMass + this.b.inverseMass + normalInertiaA +
         normalInertiaB);
    const impulse = -(1 + e) * alongNormal * massFactor;
    this.a.applyImpulse(this.normal.mul(-impulse), this.contact);
    this.b.applyImpulse(this.normal.mul(impulse), this.contact);
    // Apply friction.
    const tangent =
        relativeVelocity.sub(this.normal.mul(alongNormal)).normalized();
    const inertiaA =
        Math.pow(this.contact.sub(this.a.position).cross(tangent), 2) *
        this.a.inverseInertia;
    const inertiaB =
        Math.pow(this.contact.sub(this.b.position).cross(tangent), 2) *
        this.b.inverseInertia;
    const inertiaFactor =
        1 / (this.a.inverseMass + this.b.inverseMass + inertiaA + inertiaB);
    const unboundedFriction = -relativeVelocity.dot(tangent) * inertiaFactor;
    const staticFriction = this.a.staticFriction * this.b.staticFriction;
    const dynamicFriction = this.a.dynamicFriction * this.b.dynamicFriction;
    const frictionImpulse =
        Math.abs(unboundedFriction) < impulse * staticFriction ?
        unboundedFriction :
        -impulse * dynamicFriction;
    this.a.applyImpulse(tangent.mul(-frictionImpulse), this.contact);
    this.b.applyImpulse(tangent.mul(frictionImpulse), this.contact);
  }
  // Correct for intersections by partially removing penetration.
  correct() {
    const amount = 0.8;
    const slop = 0.01;
    const penetration = Math.max(this.depth - slop, 0);
    const factor = penetration / (this.a.inverseMass + this.b.inverseMass);
    const correction = this.normal.mul(factor * amount);
    this.a.position = this.a.position.sub(correction.mul(this.a.inverseMass));
    this.b.position = this.b.position.add(correction.mul(this.b.inverseMass));
  }
}

export function boundingRadius(points) {
  let radius = 0;
  for (const point of points) {
    const distance = point.length();
    if (distance > radius) {
      radius = distance;
    }
  }
  return radius;
}

// Points must be arranged clockwise on a canvas grid (y down).
export class Polygon {
  constructor(points, radius) {
    this.points = points;
    this.radius = radius ?? boundingRadius(points);
  }
  at(position, angle) {
    const m = Matrix.rotate(angle);
    return new Polygon(
        this.points.map(v => m.apply(v).add(position)), this.radius);
  }
  draw(context) {
    context.beginPath();
    const last = this.points[this.points.length - 1];
    context.moveTo(last.x, last.y);
    for (const v of this.points) context.lineTo(v.x, v.y);
    context.stroke();
  }
  contains(point) {
    let previous = this.points[this.points.length - 1];
    for (const v of this.points) {
      const normal = v.sub(previous).rotate90();
      if (point.sub(v).dot(normal) < 0) return false;
      previous = v;
    }
    return true;
  }
}

export class AABB extends Polygon {
  constructor(min = new Vector(-Infinity, -Infinity),
              max = new Vector(Infinity, Infinity)) {
    super([min, new Vector(max.x, min.y), max, new Vector(min.x, max.y)]);
  }
}

export class Circle extends Polygon {
  constructor(position = new Vector, radius = 1, numPoints = 32) {
    const points = [];
    for (let i = 0; i < numPoints; i++) {
      const theta = 2 * Math.PI * i / numPoints;
      const c = Math.cos(theta), s = Math.sin(theta);
      points.push(new Vector(position.x + c * radius, position.y + s * radius));
    }
    super(points);
  }
}

function support(direction, points) {
  let bestProjection = -Infinity;
  let bestPoint = null;
  for (const v of points) {
    const projection = v.dot(direction);
    if (projection > bestProjection) {
      bestProjection = projection;
      bestPoint = v;
    }
  }
  return bestPoint;
}

function leastPenetration(a, b) {
  let previous = a.points[a.points.length - 1];
  const result = {value: Infinity, contact: null, normal: null, face: []};
  for (const v of a.points) {
    const negativeNormal = v.sub(previous).rotate90().normalized();
    const s = support(negativeNormal, b.points);
    // Exit early if we have found a separating axis.
    const penetration = s.dot(negativeNormal) - v.dot(negativeNormal);
    if (penetration < 0) return null;
    if (penetration < result.value) {
      result.value = penetration;
      result.contact = s;
      result.normal = negativeNormal.neg();
      result.face = [previous, v];
    }
    previous = v;
  }
  return result;
}

export function intersect(oa, ob) {
  const offset = ob.position.sub(oa.position);
  if (offset.dot(offset) >= (oa.mesh.radius + ob.mesh.radius) ** 2) return null;
  const a = oa.getMesh(), b = ob.getMesh();
  const penetrationA = leastPenetration(a, b);
  if (!penetrationA) return null;
  const penetrationB = leastPenetration(b, a);
  if (!penetrationB) return null;
  if (penetrationA.value < penetrationB.value) {
    return new Collision(
        oa, ob, penetrationA.contact, penetrationA.normal, penetrationA.value,
        penetrationA.face);
  } else {
    return new Collision(
        ob, oa, penetrationB.contact, penetrationB.normal, penetrationB.value,
        penetrationB.face);
  }
}
