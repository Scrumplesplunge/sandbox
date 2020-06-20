import {Vector, Matrix} from './types.js';

const canvas = document.getElementById('screen');
const context = canvas.getContext('2d');

function resize() {
  canvas.width = innerWidth * devicePixelRatio;
  canvas.height = innerHeight * devicePixelRatio;
}
resize();
addEventListener('resize', resize);

export const camera = {
  position: new Vector,
  angle: 0,
  scale: 200,
};

const mouse = {
  x: 0,
  y: 0,
  down: false,
};

export const mouseHandlers = {
  move: position => {},
  up: () => {},
  down: () => {},
};

addEventListener('mousemove', event => {
  mouse.x = event.x;
  mouse.y = event.y;
  mouseHandlers.move(cameraMouse());
});

addEventListener('mousedown', event => {
  if (mouse.down) return;
  mouse.down = true;
  mouseHandlers.down();
});

addEventListener('mouseup', event => {
  if (!mouse.down) return;
  mouse.down = false;
  mouseHandlers.up();
});

addEventListener('mouseleave', event => {
  if (!mouse.down) return;
  mouse.down = false;
  mouseHandlers.up();
});

addEventListener('wheel', event => {
  const scaleHalfLife = 200;
  camera.scale *= Math.pow(0.5, event.deltaY / scaleHalfLife);
});

export function canvasMouse() {
  return new Vector(mouse.x * devicePixelRatio, mouse.y * devicePixelRatio);
}

export function cameraMouse() {
  const c = canvasMouse();
  const size = new Vector(canvas.width, canvas.height);
  const rotate = Matrix.rotate(camera.angle);
  return rotate.apply(c.sub(size.mul(0.5)).mul(1 / camera.scale))
      .add(camera.position);
}

export function draw(callback) {
  context.clearRect(0, 0, canvas.width, canvas.height);
  context.save();
  context.lineWidth = 1 / camera.scale;
  context.translate(canvas.width / 2, canvas.height / 2);
  context.scale(camera.scale, camera.scale);
  context.rotate(-camera.angle);
  context.translate(-camera.position.x, -camera.position.y);
  callback(context);
  context.restore();
}
