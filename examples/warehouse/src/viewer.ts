import {
  AmbientLight,
  Color,
  DirectionalLight,
  Fog,
  HemisphereLight,
  PerspectiveCamera,
  Scene,
  SRGBColorSpace,
  Vector3,
  WebGLRenderer,
  PCFShadowMap,
} from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

export function createViewer(container: HTMLElement) {
  const scene = new Scene();
  scene.background = new Color("#cbd2c9");
  scene.fog = new Fog("#cbd2c9", 20, 48);
  scene.add(
    new HemisphereLight("#ecf6ee", "#617471", 2.6),
    new AmbientLight("#ffffff", 0.35),
  );
  const sun = new DirectionalLight("#fff5dd", 3.5);
  sun.position.set(0, 12, 5);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  Object.assign(sun.shadow.camera, {
    left: -12,
    right: 12,
    top: 12,
    bottom: -12,
    near: 0.1,
    far: 40,
  });
  sun.shadow.normalBias = 0.025;
  sun.target.position.set(-1, 0, -2);
  scene.add(sun, sun.target);
  const renderer = new WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.outputColorSpace = SRGBColorSpace;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = PCFShadowMap;
  container.prepend(renderer.domElement);
  const view = new PerspectiveCamera(43, 1, 0.1, 100);
  const controls = new OrbitControls(view, renderer.domElement);
  controls.enableDamping = true;
  controls.maxPolarAngle = Math.PI / 2.1;
  controls.minDistance = 3;
  controls.maxDistance = 28;
  let follow = false;
  function overview() {
    follow = false;
    view.position.set(10, 11, 12);
    controls.target.set(-1.4, 0.25, -2.4);
    controls.update();
  }
  overview();
  const resize = new ResizeObserver(() => {
    const { width, height } = container.getBoundingClientRect();
    renderer.setSize(width, height);
    view.aspect = width / height;
    view.updateProjectionMatrix();
  });
  resize.observe(container);
  const followOffset = new Vector3(3, 3, 4);
  const followPosition = new Vector3();
  return {
    scene,
    renderer,
    overview,
    toggleFollow() {
      follow = !follow;
    },
    render(target: Vector3) {
      if (follow) {
        controls.target.lerp(target, 0.08);
        view.position.lerp(
          followPosition.copy(target).add(followOffset),
          0.035,
        );
      }
      controls.update();
      renderer.render(scene, view);
    },
    dispose() {
      resize.disconnect();
      controls.dispose();
      renderer.dispose();
    },
  };
}
