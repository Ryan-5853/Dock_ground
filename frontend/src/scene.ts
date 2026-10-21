import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import type { StateMessage } from "./types";

const MAX_TRAIL_POINTS = 240;

function xyzPositionToThree(
  position: [number, number, number],
): THREE.Vector3 {
  const [x, y, z] = position;
  return new THREE.Vector3(x, z, -y);
}

function xyzQuaternionToThree(
  quaternion: [number, number, number, number],
): THREE.Quaternion {
  const [w, x, y, z] = quaternion;
  const xyzToThree = new THREE.Matrix4().set(
    1, 0, 0, 0,
    0, 0, 1, 0,
    0, -1, 0, 0,
    0, 0, 0, 1,
  );
  const inverse = xyzToThree.clone().invert();
  const rotation = new THREE.Matrix4().makeRotationFromQuaternion(
    new THREE.Quaternion(x, y, z, w),
  );
  const converted = xyzToThree
    .clone()
    .multiply(rotation)
    .multiply(inverse);
  return new THREE.Quaternion().setFromRotationMatrix(converted);
}

function makeDrone(): THREE.Group {
  const drone = new THREE.Group();

  const bodyMaterial = new THREE.MeshStandardMaterial({
    color: 0x69d9ff,
    emissive: 0x0d3441,
    metalness: 0.4,
    roughness: 0.35,
  });
  const armMaterial = new THREE.MeshStandardMaterial({
    color: 0x335564,
    metalness: 0.55,
    roughness: 0.4,
  });
  const rotorMaterial = new THREE.MeshStandardMaterial({
    color: 0x18262c,
    emissive: 0x061419,
    transparent: true,
    opacity: 0.82,
  });

  const body = new THREE.Mesh(
    new THREE.BoxGeometry(0.62, 0.14, 0.42),
    bodyMaterial,
  );
  drone.add(body);

  const armGeometry = new THREE.BoxGeometry(1.05, 0.055, 0.09);
  const armA = new THREE.Mesh(armGeometry, armMaterial);
  armA.rotation.y = Math.PI / 4;
  const armB = new THREE.Mesh(armGeometry, armMaterial);
  armB.rotation.y = -Math.PI / 4;
  drone.add(armA, armB);

  const rotorGeometry = new THREE.CylinderGeometry(0.24, 0.24, 0.025, 28);
  for (const [x, z] of [
    [0.37, 0.37],
    [-0.37, 0.37],
    [0.37, -0.37],
    [-0.37, -0.37],
  ]) {
    const rotor = new THREE.Mesh(rotorGeometry, rotorMaterial);
    rotor.position.set(x, 0.06, z);
    drone.add(rotor);
  }

  const nose = new THREE.Mesh(
    new THREE.SphereGeometry(0.06, 16, 12),
    new THREE.MeshBasicMaterial({ color: 0xff5d5d }),
  );
  nose.position.set(0.36, 0.02, 0);
  drone.add(nose);

  return drone;
}

export class GroundScene {
  private readonly container: HTMLElement;
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.PerspectiveCamera(50, 1, 0.08, 250);
  private readonly renderer = new THREE.WebGLRenderer({
    antialias: true,
    powerPreference: "high-performance",
  });
  private readonly controls: OrbitControls;
  private readonly drone = makeDrone();
  private readonly targetMarker: THREE.Mesh;
  private readonly targetHalo: THREE.Mesh;
  private readonly connectionLine: THREE.Line;
  private readonly trailLine: THREE.Line;
  private readonly trailPositions = new Float32Array(
    MAX_TRAIL_POINTS * 3,
  );
  private trailCount = 0;
  private readonly currentPosition = new THREE.Vector3(0, 1.5, 0);
  private readonly desiredPosition = new THREE.Vector3(0, 1.5, 0);
  private readonly currentQuaternion = new THREE.Quaternion();
  private readonly desiredQuaternion = new THREE.Quaternion();
  private readonly targetPosition = new THREE.Vector3(0, 1.5, 0);
  private readonly resizeObserver: ResizeObserver;
  private readonly startTime = performance.now();
  private animationFrame = 0;
  private disposed = false;
  private hasReceivedPose = false;

  constructor(container: HTMLElement) {
    this.container = container;
    this.scene.background = new THREE.Color(0x071014);
    this.scene.fog = new THREE.FogExp2(0x071014, 0.018);

    this.camera.position.set(5.5, 4.2, 6.2);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 1.5, 0);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.minDistance = 2.5;
    this.controls.maxDistance = 30;
    this.controls.maxPolarAngle = Math.PI * 0.49;

    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.domElement.className = "scene-canvas";
    container.appendChild(this.renderer.domElement);

    this.scene.add(new THREE.HemisphereLight(0xbfefff, 0x10181b, 1.7));
    const keyLight = new THREE.DirectionalLight(0xffffff, 2.0);
    keyLight.position.set(4, 8, 2);
    this.scene.add(keyLight);

    const grid = new THREE.GridHelper(30, 30, 0x2a5966, 0x16323b);
    grid.position.y = 0;
    this.scene.add(grid);

    const floor = new THREE.Mesh(
      new THREE.CircleGeometry(15, 64),
      new THREE.MeshBasicMaterial({
        color: 0x09171c,
        transparent: true,
        opacity: 0.86,
      }),
    );
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = -0.01;
    this.scene.add(floor);

    this.scene.add(
      new THREE.ArrowHelper(
        new THREE.Vector3(1, 0, 0),
        new THREE.Vector3(0, 0, 0),
        2.4,
        0x5de1a5,
        0.35,
        0.2,
      ),
    );
    this.scene.add(
      new THREE.ArrowHelper(
        new THREE.Vector3(0, 0, -1),
        new THREE.Vector3(0, 0, 0),
        2.4,
        0x5ab9e8,
        0.35,
        0.2,
      ),
    );
    this.scene.add(
      new THREE.ArrowHelper(
        new THREE.Vector3(0, 1, 0),
        new THREE.Vector3(0, 0, 0),
        2.4,
        0xf0c75d,
        0.35,
        0.2,
      ),
    );

    this.drone.position.copy(this.currentPosition);
    this.scene.add(this.drone);

    this.targetMarker = new THREE.Mesh(
      new THREE.IcosahedronGeometry(0.12, 2),
      new THREE.MeshBasicMaterial({ color: 0x62f0b4 }),
    );
    this.targetHalo = new THREE.Mesh(
      new THREE.RingGeometry(0.24, 0.3, 32),
      new THREE.MeshBasicMaterial({
        color: 0x62f0b4,
        transparent: true,
        opacity: 0.5,
        side: THREE.DoubleSide,
      }),
    );
    this.targetHalo.rotation.x = -Math.PI / 2;
    this.scene.add(this.targetMarker, this.targetHalo);

    const lineMaterial = new THREE.LineDashedMaterial({
      color: 0x4db7c8,
      dashSize: 0.12,
      gapSize: 0.08,
      transparent: true,
      opacity: 0.55,
    });
    const lineGeometry = new THREE.BufferGeometry().setFromPoints([
      this.currentPosition,
      this.targetPosition,
    ]);
    this.connectionLine = new THREE.Line(lineGeometry, lineMaterial);
    this.connectionLine.computeLineDistances();
    this.scene.add(this.connectionLine);

    const trailGeometry = new THREE.BufferGeometry();
    trailGeometry.setAttribute(
      "position",
      new THREE.BufferAttribute(this.trailPositions, 3),
    );
    trailGeometry.setDrawRange(0, 0);
    this.trailLine = new THREE.Line(
      trailGeometry,
      new THREE.LineBasicMaterial({
        color: 0x55d9e8,
        transparent: true,
        opacity: 0.72,
      }),
    );
    this.scene.add(this.trailLine);

    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(container);
    this.resize();
    this.animate();
  }

  update(state: StateMessage): void {
    this.desiredPosition.copy(
      xyzPositionToThree(state.pose.position_xyz_m),
    );
    this.desiredQuaternion.copy(
      xyzQuaternionToThree(state.pose.orientation_wxyz),
    );
    this.targetPosition.copy(
      xyzPositionToThree(state.target.position_xyz_m),
    );
    if (!this.hasReceivedPose) {
      const cameraOffset = this.camera.position
        .clone()
        .sub(this.controls.target);
      this.currentPosition.copy(this.desiredPosition);
      this.controls.target.copy(this.desiredPosition);
      this.camera.position.copy(this.desiredPosition).add(cameraOffset);
      this.hasReceivedPose = true;
    }
    this.targetMarker.position.copy(this.targetPosition);
    this.targetHalo.position.copy(this.targetPosition);
  }

  dispose(): void {
    this.disposed = true;
    cancelAnimationFrame(this.animationFrame);
    this.resizeObserver.disconnect();
    this.controls.dispose();
    this.scene.traverse((object) => {
      if (object instanceof THREE.Mesh || object instanceof THREE.Line) {
        object.geometry.dispose();
        if (Array.isArray(object.material)) {
          object.material.forEach((material) => material.dispose());
        } else {
          object.material.dispose();
        }
      }
    });
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }

  private resize(): void {
    const width = Math.max(1, this.container.clientWidth);
    const height = Math.max(1, this.container.clientHeight);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height, false);
  }

  private pushTrailPoint(): void {
    if (this.trailCount > 0) {
      const index = (this.trailCount - 1) * 3;
      const dx = this.currentPosition.x - this.trailPositions[index];
      const dy = this.currentPosition.y - this.trailPositions[index + 1];
      const dz = this.currentPosition.z - this.trailPositions[index + 2];
      if (dx * dx + dy * dy + dz * dz < 0.0025) {
        return;
      }
    }
    if (this.trailCount >= MAX_TRAIL_POINTS) {
      this.trailPositions.copyWithin(
        0,
        3,
        this.trailPositions.length,
      );
      this.trailCount = MAX_TRAIL_POINTS - 1;
    }
    const index = this.trailCount * 3;
    this.trailPositions[index] = this.currentPosition.x;
    this.trailPositions[index + 1] = this.currentPosition.y;
    this.trailPositions[index + 2] = this.currentPosition.z;
    ++this.trailCount;
    const geometry = this.trailLine.geometry;
    geometry.setDrawRange(0, this.trailCount);
    const attribute = geometry.getAttribute("position");
    attribute.needsUpdate = true;
  }

  private animate = (): void => {
    if (this.disposed) {
      return;
    }
    this.animationFrame = requestAnimationFrame(this.animate);
    const elapsed = (performance.now() - this.startTime) * 0.001;
    this.currentPosition.lerp(this.desiredPosition, 0.16);
    this.currentQuaternion.slerp(this.desiredQuaternion, 0.18);
    this.drone.position.copy(this.currentPosition);
    this.drone.quaternion.copy(this.currentQuaternion);

    this.targetHalo.scale.setScalar(1 + Math.sin(elapsed * 4) * 0.08);
    const positions = this.connectionLine.geometry.getAttribute("position");
    positions.setXYZ(
      0,
      this.currentPosition.x,
      this.currentPosition.y,
      this.currentPosition.z,
    );
    positions.setXYZ(
      1,
      this.targetPosition.x,
      this.targetPosition.y,
      this.targetPosition.z,
    );
    positions.needsUpdate = true;
    this.connectionLine.computeLineDistances();
    this.pushTrailPoint();
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  };
}
