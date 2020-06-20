export class Vector {
  constructor(x = 0, y = 0) {
    this.x = x;
    this.y = y;
  }
  clone() { return new Vector(this.x, this.y) }
  add(v) { return new Vector(this.x + v.x, this.y + v.y) }
  sub(v) { return new Vector(this.x - v.x, this.y - v.y) }
  mul(s) { return new Vector(this.x * s, this.y * s) }
  dot(v) { return this.x * v.x + this.y * v.y }
  cross(v) { return this.x * v.y - this.y * v.x }
  rotate90() { return new Vector(-this.y, this.x) }
  length() { return Math.sqrt(this.dot(this)) }
  neg() { return new Vector(-this.x, -this.y) }
  normalized() {
    const length = this.length();
    return length == 0 ? new Vector(1, 0) : this.mul(1 / length);
  }
  toString() {
    return `[${this.x.toPrecision(4)}, ${this.y.toPrecision(4)}]`;
  }
}

export class Matrix {
  static rotate(theta) {
    const c = Math.cos(theta), s = Math.sin(theta);
    return new Matrix(new Vector(c, s), new Vector(-s, c));
  }
  constructor(a, b) {
    this.a = a;
    this.b = b;
  }
  apply(v) {
    return this.a.mul(v.x).add(this.b.mul(v.y));
  }
}
