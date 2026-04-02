
import React from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { Activity } from 'lucide-react';
import loadMujoco from 'mujoco-js';
import URDFLoader, { URDFJoint, URDFRobot } from 'urdf-loader';
import {
  PlanningTrajectory,
  RobotAsset,
  RobotMaterialProfile,
  RobotRuntime,
  RobotSourceType,
} from '../types';

type LoadState = 'loading' | 'ready' | 'error';

type MujocoModel = {
  nbody: number;
  ngeom: number;
  geom_group: Int32Array;
  geom_type: Int32Array;
  geom_size: Float64Array;
  geom_pos: Float64Array;
  geom_quat: Float64Array;
  geom_matid: Int32Array;
  mat_rgba: Float32Array;
  geom_rgba: Float32Array;
  geom_dataid: Int32Array;
  mesh_vertadr: Int32Array;
  mesh_vertnum: Int32Array;
  mesh_faceadr: Int32Array;
  mesh_facenum: Int32Array;
  mesh_vert: Float32Array;
  mesh_face: Int32Array;
  geom_bodyid: Int32Array;
  delete: () => void;
  [key: string]: unknown;
};

type MujocoData = {
  qpos: Float64Array;
  xpos: Float64Array;
  xquat: Float64Array;
  delete: () => void;
  [key: string]: unknown;
};

type MujocoModule = {
  MjModel: { loadFromXML: (path: string) => MujocoModel };
  MjData: new (model: MujocoModel) => MujocoData;
  mj_forward: (model: MujocoModel, data: MujocoData) => void;
  mjtGeom: Record<string, number | { value: number }>;
  MEMFS: unknown;
  FS: {
    mkdir: (path: string) => void;
    mount: (type: unknown, opts: { root: string }, path: string) => void;
    unmount: (path: string) => void;
    writeFile: (path: string, content: string | Uint8Array) => void;
  };
};

type ResolvedRobotSource = {
  runtime: Exclude<RobotRuntime, 'auto'>;
  sourceType: Exclude<RobotSourceType, 'AUTO'>;
  sourcePath: string;
};

interface SimulationViewProps {
  robot: RobotAsset;
  trajectory?: PlanningTrajectory | null;
  currentTimeSec?: number;
}

function toAbsoluteAssetUrl(publicPath: string) {
  return new URL(publicPath.replace(/^\//, ''), document.baseURI).toString();
}

function getFileName(publicPath: string) {
  const normalized = publicPath.replace(/\\/g, '/');
  const tokens = normalized.split('/').filter(Boolean);
  return tokens.length ? tokens[tokens.length - 1] : normalized;
}

function getDirName(path: string) {
  const normalized = path.replace(/\\/g, '/');
  const idx = normalized.lastIndexOf('/');
  if (idx <= 0) return '';
  return normalized.slice(0, idx);
}

function ensurePosix(path: string) {
  return path.replace(/\\/g, '/');
}

function inferSourceType(path: string): Exclude<RobotSourceType, 'AUTO'> {
  const lower = path.toLowerCase();
  if (lower.endsWith('.mjcf.xml') || lower.endsWith('.mjcf')) {
    return 'MJCF';
  }
  if (lower.endsWith('.urdf') || lower.endsWith('.urdf.xml')) {
    return 'URDF';
  }
  return 'URDF';
}

function runtimeBySourceType(_: Exclude<RobotSourceType, 'AUTO'>): Exclude<RobotRuntime, 'auto'> {
  return 'mujoco';
}

function sourcePriority(path: string) {
  const lower = path.toLowerCase();
  if (lower.endsWith('.mjcf.xml') || lower.endsWith('.mjcf')) return 0;
  if (lower.endsWith('.urdf.xml')) return 3;
  if (lower.endsWith('.urdf')) return 2;
  if (lower.endsWith('.xml')) return 1;
  return 99;
}

async function isAssetReachable(publicPath: string) {
  const url = toAbsoluteAssetUrl(publicPath);
  try {
    const response = await fetch(url, { cache: 'no-store' });
    if (!response.ok) {
      return false;
    }
    const contentType = response.headers.get('content-type')?.toLowerCase() ?? '';
    if (contentType.includes('text/html')) {
      return false;
    }
    return true;
  } catch {
    return false;
  }
}

function inferSourceTypeFromText(
  text: string,
  fallback: Exclude<RobotSourceType, 'AUTO'>,
): Exclude<RobotSourceType, 'AUTO'> {
  const sample = text.slice(0, 4096).toLowerCase();
  if (sample.includes('<mujoco')) {
    return 'MJCF';
  }
  if (sample.includes('<robot')) {
    return 'URDF';
  }
  return fallback;
}

function isLikelyValidSourceText(text: string) {
  const sample = text.slice(0, 4096).toLowerCase();
  if (sample.includes('<!doctype html') || sample.includes('<html')) {
    return false;
  }
  return sample.includes('<mujoco') || sample.includes('<robot');
}

async function resolveRobotSource(robot: RobotAsset): Promise<ResolvedRobotSource> {
  if (robot.runtime !== 'auto') {
    const sourceType = inferSourceType(robot.sourcePath);
    return {
      runtime: runtimeBySourceType(sourceType),
      sourceType,
      sourcePath: robot.sourcePath,
    };
  }

  const candidates = Array.from(
    new Set([...(robot.sourceCandidates ?? []), robot.sourcePath].map((item) => item.trim()).filter(Boolean)),
  ).sort((a, b) => sourcePriority(a) - sourcePriority(b));

  for (const candidate of candidates) {
    // Browser cannot list local public directory, so probe candidates directly.
    const sourceTypeFromPath = inferSourceType(candidate);
    // eslint-disable-next-line no-await-in-loop
    try {
      const response = await fetch(toAbsoluteAssetUrl(candidate), { cache: 'no-store' });
      if (!response.ok) {
        continue;
      }
      const contentType = response.headers.get('content-type')?.toLowerCase() ?? '';
      if (contentType.includes('text/html')) {
        continue;
      }
      const bodyText = await response.text();
      if (!isLikelyValidSourceText(bodyText)) {
        continue;
      }
      const sourceType = inferSourceTypeFromText(bodyText, sourceTypeFromPath);
      return {
        runtime: runtimeBySourceType(sourceType),
        sourceType,
        sourcePath: candidate,
      };
    } catch {
      continue;
    }
  }

  throw new Error(`No model source found for ${robot.name} in ${robot.publicRoot}`);
}

function resolveDisplayScale(robot: RobotAsset) {
  const raw = robot.displayScale ?? 1;
  if (Array.isArray(raw)) {
    return new THREE.Vector3(
      raw[0] > 0 ? raw[0] : 1,
      raw[1] > 0 ? raw[1] : 1,
      raw[2] > 0 ? raw[2] : 1,
    );
  }
  const uniform = raw > 0 ? raw : 1;
  return new THREE.Vector3(uniform, uniform, uniform);
}

function resolveDisplayOffset(robot: RobotAsset) {
  const offset = robot.displayOffset ?? [0, 0, 0];
  return new THREE.Vector3(offset[0], offset[1], offset[2]);
}

function getFallbackJointValues(robot: RobotAsset) {
  if (robot.defaultJointValues.length > 0) {
    return [...robot.defaultJointValues];
  }
  return Array.from({ length: Math.max(robot.jointCount, 1) }, () => 0);
}

function sampleTrajectory(trajectory: PlanningTrajectory | null | undefined, timeSec: number): number[] | null {
  if (!trajectory) return null;
  const times = trajectory.tUniform;
  const qUniform = trajectory.qUniform;
  if (times.length === 0 || qUniform.length === 0 || times.length !== qUniform.length) return null;

  if (timeSec <= times[0]) return [...qUniform[0]];
  const last = times.length - 1;
  if (timeSec >= times[last]) return [...qUniform[last]];

  let low = 0;
  let high = last;
  while (high - low > 1) {
    const mid = Math.floor((low + high) / 2);
    if (times[mid] <= timeSec) {
      low = mid;
    } else {
      high = mid;
    }
  }

  const t0 = times[low];
  const t1 = times[high];
  const alpha = t1 > t0 ? (timeSec - t0) / (t1 - t0) : 0;
  const q0 = qUniform[low];
  const q1 = qUniform[high];
  const dof = Math.min(q0.length, q1.length);
  const out = new Array(dof);
  for (let i = 0; i < dof; i += 1) {
    out[i] = q0[i] + (q1[i] - q0[i]) * alpha;
  }
  return out;
}

function getDisplayedJointValues(robot: RobotAsset, trajectory: PlanningTrajectory | null | undefined, timeSec: number) {
  const fallback = getFallbackJointValues(robot);
  const sampled = sampleTrajectory(trajectory, timeSec);
  const count = Math.max(robot.jointCount, fallback.length, 1);
  return Array.from({ length: count }, (_, index) => sampled?.[index] ?? fallback[index] ?? 0);
}

function ensureVirtualDirectory(mujoco: MujocoModule, path: string) {
  const tokens = ensurePosix(path).split('/').filter(Boolean);
  let current = '';
  for (const token of tokens) {
    current += `/${token}`;
    try {
      mujoco.FS.mkdir(current);
    } catch {
      // Existing folder.
    }
  }
}

function toVirtualPath(publicRoot: string, publicPath: string) {
  const root = ensurePosix(publicRoot).replace(/\/$/, '');
  const path = ensurePosix(publicPath);
  const prefix = `${root}/`;
  if (path.startsWith(prefix)) {
    return `/working/${path.slice(prefix.length)}`;
  }
  return `/working/${getFileName(path)}`;
}
function resolvePublicPath(baseFilePath: string, relativePath: string) {
  if (/^(https?:)?\/\//i.test(relativePath)) {
    return relativePath;
  }
  if (relativePath.startsWith('/')) {
    return relativePath;
  }
  const baseDir = getDirName(baseFilePath);
  return new URL(relativePath, toAbsoluteAssetUrl(`${baseDir}/`)).pathname;
}

function getUrdfMeshReferences(urdfText: string) {
  const refs: string[] = [];
  const regex = /filename\s*=\s*["']([^"']+)["']/gi;
  for (const match of urdfText.matchAll(regex)) {
    const ref = match[1]?.trim();
    if (ref) refs.push(ref);
  }
  return refs;
}

async function normalizeUrdfPaths(
  robot: RobotAsset,
  urdfText: string,
) {
  let normalized = urdfText;
  let rewritten = false;
  const normalizedRoot = robot.publicRoot.replace(/\/$/, '');

  for (const ref of getUrdfMeshReferences(urdfText)) {
    if (!/^package:\/\//i.test(ref)) {
      continue;
    }

    const relative = ref.replace(/^package:\/\/[^/]+\//i, '');
    const expectedPath = `${normalizedRoot}/${relative}`;
    // eslint-disable-next-line no-await-in-loop
    const expectedExists = await isAssetReachable(expectedPath);
    if (expectedExists) {
      continue;
    }

    const fallbackFile = getFileName(relative);
    const flattenedPath = `${normalizedRoot}/${fallbackFile}`;
    // eslint-disable-next-line no-await-in-loop
    const flattenedExists = await isAssetReachable(flattenedPath);
    if (!flattenedExists) {
      continue;
    }

    normalized = normalized.split(ref).join(`./${fallbackFile}`);
    rewritten = true;
  }

  return {
    text: normalized,
    rewritten,
  };
}

async function resolveUrdfDependencyPath(
  robot: RobotAsset,
  sourcePath: string,
  rawRef: string,
) {
  const normalizedRoot = robot.publicRoot.replace(/\/$/, '');
  const normalizedRef = rawRef.trim();

  const candidates: string[] = [];
  if (/^package:\/\//i.test(normalizedRef)) {
    const relative = normalizedRef.replace(/^package:\/\/[^/]+\//i, '');
    candidates.push(`${normalizedRoot}/${relative}`);
    candidates.push(`${normalizedRoot}/${getFileName(relative)}`);
  } else if (normalizedRef.startsWith('/')) {
    candidates.push(normalizedRef);
  } else {
    const baseDir = getDirName(sourcePath);
    candidates.push(`${baseDir}/${normalizedRef}`);
    candidates.push(`${normalizedRoot}/${normalizedRef}`);
    candidates.push(`${normalizedRoot}/${getFileName(normalizedRef)}`);
  }

  for (const candidate of candidates) {
    // eslint-disable-next-line no-await-in-loop
    if (await isAssetReachable(candidate)) {
      return candidate;
    }
  }
  return null;
}

async function normalizeUrdfForMujoco(
  robot: RobotAsset,
  sourcePath: string,
  urdfText: string,
) {
  let normalized = urdfText;
  const normalizedRoot = robot.publicRoot.replace(/\/$/, '');
  const dependencies = new Map<string, string>();

  for (const ref of getUrdfMeshReferences(urdfText)) {
    // eslint-disable-next-line no-await-in-loop
    const resolved = await resolveUrdfDependencyPath(robot, sourcePath, ref);
    if (!resolved) {
      continue;
    }

    let relative = getFileName(resolved);
    const normalizedResolved = ensurePosix(resolved);
    const rootPrefix = `${ensurePosix(normalizedRoot)}/`;
    if (normalizedResolved.startsWith(rootPrefix)) {
      relative = normalizedResolved.slice(rootPrefix.length);
    }

    normalized = normalized.split(ref).join(`./${relative}`);
    dependencies.set(resolved, relative);
  }

  return {
    text: normalized,
    dependencies,
  };
}

function extractMjcfFileReferences(xmlText: string) {
  const refs: string[] = [];
  const regex = /\bfile\s*=\s*["']([^"']+)["']/gi;
  for (const match of xmlText.matchAll(regex)) {
    const ref = match[1]?.trim();
    if (ref) refs.push(ref);
  }
  return refs;
}

function extractMjcfMeshDir(xmlText: string) {
  const compilerTagMatch = xmlText.match(/<compiler\b[^>]*>/i);
  if (!compilerTagMatch) return null;
  const meshDirMatch = compilerTagMatch[0].match(/\bmeshdir\s*=\s*["']([^"']+)["']/i);
  if (!meshDirMatch?.[1]) return null;
  const normalized = ensurePosix(meshDirMatch[1]).replace(/^\.\//, '').replace(/^\/+/, '').replace(/\/+$/, '');
  return normalized || null;
}

function shouldTreatAsRelativeMjcfRef(ref: string) {
  if (!ref.trim()) return false;
  if (ref.startsWith('/')) return false;
  if (/^(https?:)?\/\//i.test(ref)) return false;
  if (/^[A-Za-z]:[\\/]/.test(ref)) return false;
  return true;
}

async function populateVirtualFileSystem(mujoco: MujocoModule, robot: RobotAsset, source: ResolvedRobotSource) {
  try {
    mujoco.FS.unmount('/working');
  } catch {
    // Ignore when not mounted.
  }
  ensureVirtualDirectory(mujoco, '/working');
  mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');

  const sourceVirtualPath = toVirtualPath(robot.publicRoot, source.sourcePath);
  const explicitAssets = (robot.mujocoAssetFiles ?? []).map((item) => (
    item.startsWith('/') ? item : `${robot.publicRoot.replace(/\/$/, '')}/${item}`
  ));

  if (source.sourceType === 'URDF') {
    const sourceResponse = await fetch(toAbsoluteAssetUrl(source.sourcePath));
    if (!sourceResponse.ok) {
      throw new Error(`Failed to load URDF source: ${source.sourcePath}`);
    }
    const sourceText = await sourceResponse.text();
    const normalizedUrdf = await normalizeUrdfForMujoco(robot, source.sourcePath, sourceText);

    ensureVirtualDirectory(mujoco, getDirName(sourceVirtualPath));
    mujoco.FS.writeFile(sourceVirtualPath, normalizedUrdf.text);

    for (const [publicPath, relativeTarget] of normalizedUrdf.dependencies.entries()) {
      const depResponse = await fetch(toAbsoluteAssetUrl(publicPath));
      if (!depResponse.ok) {
        throw new Error(`Failed to load URDF dependency: ${publicPath}`);
      }
      const target = `/working/${ensurePosix(relativeTarget)}`;
      ensureVirtualDirectory(mujoco, getDirName(target));
      mujoco.FS.writeFile(target, new Uint8Array(await depResponse.arrayBuffer()));
    }

    return sourceVirtualPath;
  }

  if (explicitAssets.length > 0) {
    const assets = new Set([...explicitAssets, source.sourcePath]);
    for (const publicPath of assets) {
      const response = await fetch(toAbsoluteAssetUrl(publicPath));
      if (!response.ok) {
        throw new Error(`Failed to load MuJoCo asset: ${publicPath}`);
      }
      const target = toVirtualPath(robot.publicRoot, publicPath);
      ensureVirtualDirectory(mujoco, getDirName(target));
      if (publicPath.toLowerCase().endsWith('.xml')) {
        mujoco.FS.writeFile(target, await response.text());
      } else {
        mujoco.FS.writeFile(target, new Uint8Array(await response.arrayBuffer()));
      }
    }
    return sourceVirtualPath;
  }

  // Fallback for auto-added MJCF assets: parse XML file references and fetch them.
  const xmlQueue: string[] = [source.sourcePath];
  const loadedXml = new Set<string>();
  const binaryTargetsByPublicPath = new Map<string, Set<string>>();

  while (xmlQueue.length > 0) {
    const xmlPath = xmlQueue.shift()!;
    if (loadedXml.has(xmlPath)) continue;

    const response = await fetch(toAbsoluteAssetUrl(xmlPath));
    if (!response.ok) {
      throw new Error(`Failed to load MuJoCo XML: ${xmlPath}`);
    }

    const xmlText = await response.text();
    loadedXml.add(xmlPath);

    const target = toVirtualPath(robot.publicRoot, xmlPath);
    ensureVirtualDirectory(mujoco, getDirName(target));
    mujoco.FS.writeFile(target, xmlText);
    const meshDir = extractMjcfMeshDir(xmlText);

    for (const ref of extractMjcfFileReferences(xmlText)) {
      const resolved = resolvePublicPath(xmlPath, ref);
      if (resolved.toLowerCase().endsWith('.xml')) {
        if (!loadedXml.has(resolved)) {
          xmlQueue.push(resolved);
        }
      } else {
        const targets = binaryTargetsByPublicPath.get(resolved) ?? new Set<string>();
        targets.add(toVirtualPath(robot.publicRoot, resolved));

        if (meshDir && shouldTreatAsRelativeMjcfRef(ref)) {
          const refPosix = ensurePosix(ref).replace(/^\.\//, '').replace(/^\/+/, '');
          const meshDirTarget = `/working/${meshDir}/${refPosix}`;
          targets.add(meshDirTarget);
        }

        binaryTargetsByPublicPath.set(resolved, targets);
      }
    }
  }

  for (const [binaryPath, targets] of binaryTargetsByPublicPath.entries()) {
    const response = await fetch(toAbsoluteAssetUrl(binaryPath));
    if (!response.ok) {
      throw new Error(`Failed to load MuJoCo dependency: ${binaryPath}`);
    }
    const content = new Uint8Array(await response.arrayBuffer());

    for (const target of targets) {
      ensureVirtualDirectory(mujoco, getDirName(target));
      mujoco.FS.writeFile(target, content);
    }
  }

  return sourceVirtualPath;
}

function createFloorTexture() {
  const canvas = document.createElement('canvas');
  canvas.width = 256;
  canvas.height = 256;
  const context = canvas.getContext('2d');
  if (!context) return null;

  const tile = 64;
  for (let y = 0; y < canvas.height; y += tile) {
    for (let x = 0; x < canvas.width; x += tile) {
      const even = ((x / tile) + (y / tile)) % 2 === 0;
      context.fillStyle = even ? '#4d77a8' : '#345f91';
      context.fillRect(x, y, tile, tile);
    }
  }

  context.strokeStyle = 'rgba(225, 238, 255, 0.64)';
  context.lineWidth = 2.5;
  for (let i = 0; i <= canvas.width; i += tile) {
    context.beginPath();
    context.moveTo(i, 0);
    context.lineTo(i, canvas.height);
    context.stroke();
    context.beginPath();
    context.moveTo(0, i);
    context.lineTo(canvas.width, i);
    context.stroke();
  }

  const texture = new THREE.CanvasTexture(canvas);
  texture.wrapS = THREE.RepeatWrapping;
  texture.wrapT = THREE.RepeatWrapping;
  texture.repeat.set(18, 18);
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

function styleStandardMaterial(
  material: THREE.MeshStandardMaterial | THREE.MeshPhysicalMaterial,
  profile: RobotMaterialProfile,
) {
  const color = material.color ?? new THREE.Color(0.8, 0.8, 0.8);
  const isOrange = color.r > 0.75 && color.g > 0.25 && color.g < 0.62 && color.b < 0.22;
  const luminance = 0.2126 * color.r + 0.7152 * color.g + 0.0722 * color.b;
  const isDark = luminance < 0.25;

  material.side = THREE.DoubleSide;
  material.emissive = material.emissive ?? new THREE.Color(0x000000);
  material.envMapIntensity = 1.25;
  material.flatShading = false;
  material.needsUpdate = true;

  if (profile === 'industrial_orange_dark') {
    material.roughness = isOrange ? 0.22 : isDark ? 0.68 : 0.28;
    material.metalness = isOrange ? 0.24 : isDark ? 0.3 : 0.78;
    material.emissive.copy(isOrange ? new THREE.Color('#2c1200') : new THREE.Color('#000000'));
    material.emissiveIntensity = isOrange ? 0.04 : 0.008;
  } else {
    material.roughness = isDark ? 0.52 : 0.3;
    material.metalness = isDark ? 0.42 : 0.74;
    material.emissive.copy(new THREE.Color('#05080c'));
    material.emissiveIntensity = isDark ? 0.018 : 0.01;
  }

  if (material instanceof THREE.MeshPhysicalMaterial) {
    material.clearcoat = profile === 'industrial_orange_dark' ? 0.74 : 0.66;
    material.clearcoatRoughness = isDark ? 0.35 : 0.2;
    material.sheen = 0.08;
    material.sheenRoughness = 0.26;
    material.reflectivity = 0.72;
  }
}

function enhanceMaterial(material: THREE.Material, profile: RobotMaterialProfile) {
  if (material instanceof THREE.MeshStandardMaterial || material instanceof THREE.MeshPhysicalMaterial) {
    styleStandardMaterial(material, profile);
    return material;
  }

  const anyMaterial = material as THREE.Material & {
    color?: THREE.Color;
    map?: THREE.Texture;
    transparent?: boolean;
    opacity?: number;
  };

  const converted = new THREE.MeshPhysicalMaterial({
    color: anyMaterial.color?.clone?.() ?? new THREE.Color(0xbfc7d2),
    map: anyMaterial.map ?? null,
    transparent: anyMaterial.transparent ?? false,
    opacity: anyMaterial.opacity ?? 1,
  });
  styleStandardMaterial(converted, profile);
  material.dispose?.();
  return converted;
}

function applyUrdfVisualProfile(robot: URDFRobot, profile: RobotMaterialProfile) {
  robot.traverse((object) => {
    const mesh = object as THREE.Mesh;
    if (!(mesh as unknown as { isMesh?: boolean }).isMesh) return;

    mesh.castShadow = true;
    mesh.receiveShadow = true;

    if (Array.isArray(mesh.material)) {
      mesh.material = mesh.material.map((item) => enhanceMaterial(item, profile));
    } else if (mesh.material) {
      mesh.material = enhanceMaterial(mesh.material, profile);
    }
  });
}
function buildGeom(
  module: MujocoModule,
  model: MujocoModel,
  geomIndex: number,
  profile: RobotMaterialProfile,
) {
  if (model.geom_group?.[geomIndex] === 3) return null;

  const type = model.geom_type[geomIndex];
  const size = model.geom_size.subarray(geomIndex * 3, geomIndex * 3 + 3);
  const pos = model.geom_pos.subarray(geomIndex * 3, geomIndex * 3 + 3);
  const quat = model.geom_quat.subarray(geomIndex * 4, geomIndex * 4 + 4);
  const materialId = model.geom_matid[geomIndex];
  const rgba = materialId >= 0
    ? model.mat_rgba.subarray(materialId * 4, materialId * 4 + 4)
    : model.geom_rgba.subarray(geomIndex * 4, geomIndex * 4 + 4);

  const getEnum = (value: unknown) => (value as { value?: number })?.value ?? value;
  const geomTypes = module.mjtGeom;

  let geometry: THREE.BufferGeometry | null = null;

  if (type === getEnum(geomTypes.mjGEOM_PLANE)) {
    return null;
  }
  if (type === getEnum(geomTypes.mjGEOM_BOX)) {
    geometry = new THREE.BoxGeometry(size[0] * 2, size[1] * 2, size[2] * 2);
  } else if (type === getEnum(geomTypes.mjGEOM_SPHERE)) {
    geometry = new THREE.SphereGeometry(size[0], 24, 24);
  } else if (type === getEnum(geomTypes.mjGEOM_CYLINDER)) {
    geometry = new THREE.CylinderGeometry(size[0], size[0], size[1] * 2, 24);
    geometry.rotateX(Math.PI / 2);
  } else if (type === getEnum(geomTypes.mjGEOM_CAPSULE)) {
    geometry = new THREE.CapsuleGeometry(size[0], size[1] * 2, 10, 20);
    geometry.rotateX(Math.PI / 2);
  } else if (type === getEnum(geomTypes.mjGEOM_MESH)) {
    const meshId = model.geom_dataid[geomIndex];
    const vertStart = model.mesh_vertadr[meshId];
    const vertCount = model.mesh_vertnum[meshId];
    const faceStart = model.mesh_faceadr[meshId];
    const faceCount = model.mesh_facenum[meshId];

    geometry = new THREE.BufferGeometry();
    geometry.setAttribute(
      'position',
      new THREE.Float32BufferAttribute(
        model.mesh_vert.subarray(vertStart * 3, (vertStart + vertCount) * 3),
        3,
      ),
    );
    geometry.setIndex(Array.from(model.mesh_face.subarray(faceStart * 3, (faceStart + faceCount) * 3)));
    geometry.computeVertexNormals();
  }

  if (!geometry) return null;

  const material = new THREE.MeshPhysicalMaterial({
    color: new THREE.Color(rgba[0], rgba[1], rgba[2]),
    transparent: rgba[3] < 1,
    opacity: rgba[3],
  });
  styleStandardMaterial(material, profile);

  const mesh = new THREE.Mesh(geometry, material);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  mesh.position.set(pos[0], pos[1], pos[2]);
  mesh.quaternion.set(quat[1], quat[2], quat[3], quat[0]);
  return mesh;
}

function disposeSceneResources(scene: THREE.Scene) {
  scene.traverse((object) => {
    const mesh = object as THREE.Mesh & {
      geometry?: THREE.BufferGeometry;
      material?: THREE.Material | THREE.Material[];
    };
    mesh.geometry?.dispose?.();
    if (Array.isArray(mesh.material)) {
      mesh.material.forEach((item) => item.dispose?.());
    } else {
      mesh.material?.dispose?.();
    }
  });
}

function createViewport(container: HTMLDivElement) {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color('#6a90bc');
  scene.fog = new THREE.Fog('#7da0c9', 7, 26);

  const camera = new THREE.PerspectiveCamera(42, container.clientWidth / Math.max(container.clientHeight, 1), 0.01, 100);
  camera.position.set(2.4, -1.8, 1.9);
  camera.up.set(0, 0, 1);

  const renderer = new THREE.WebGLRenderer({ antialias: false, alpha: true, powerPreference: 'default' });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1));
  renderer.setSize(container.clientWidth, container.clientHeight);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFShadowMap;
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.2;
  container.innerHTML = '';
  container.appendChild(renderer.domElement);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = false;

  scene.add(new THREE.HemisphereLight(0xdbe9ff, 0x46678f, 1.06));

  const keyLight = new THREE.DirectionalLight(0xfff2d8, 2.1);
  keyLight.position.set(4.1, -3.0, 7.2);
  keyLight.castShadow = true;
  keyLight.shadow.mapSize.set(1024, 1024);
  keyLight.shadow.bias = -0.0002;
  keyLight.shadow.normalBias = 0.02;
  keyLight.shadow.camera.near = 0.5;
  keyLight.shadow.camera.far = 20;
  keyLight.shadow.camera.left = -4;
  keyLight.shadow.camera.right = 4;
  keyLight.shadow.camera.top = 4;
  keyLight.shadow.camera.bottom = -4;
  scene.add(keyLight);

  const fillLight = new THREE.DirectionalLight(0x9dcaff, 0.52);
  fillLight.position.set(-4.2, 2.7, 4.0);
  scene.add(fillLight);

  const rimLight = new THREE.DirectionalLight(0xffffff, 0.3);
  rimLight.position.set(1.2, 4.5, 3.0);
  scene.add(rimLight);

  const floorTexture = createFloorTexture();
  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(24, 24),
    new THREE.MeshPhysicalMaterial({
      map: floorTexture ?? undefined,
      transparent: true,
      opacity: 0.93,
      roughness: 0.2,
      metalness: 0.04,
      clearcoat: 0.22,
      clearcoatRoughness: 0.34,
      reflectivity: 0.18,
    }),
  );
  floor.position.z = -0.0015;
  floor.receiveShadow = true;
  scene.add(floor);

  const resize = () => {
    camera.aspect = container.clientWidth / Math.max(container.clientHeight, 1);
    camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, container.clientHeight);
  };

  window.addEventListener('resize', resize);
  resize();

  return {
    scene,
    camera,
    renderer,
    controls,
    dispose: () => {
      window.removeEventListener('resize', resize);
      controls.dispose();
      floorTexture?.dispose?.();
      disposeSceneResources(scene);
      renderer.dispose();
      renderer.domElement.remove();
    },
  };
}

function applyDisplayTransform(displayRoot: THREE.Group, robot: RobotAsset) {
  displayRoot.scale.copy(resolveDisplayScale(robot));
  displayRoot.position.copy(resolveDisplayOffset(robot));
}

function fitCameraToObject(camera: THREE.PerspectiveCamera, controls: OrbitControls, object: THREE.Object3D, robot: RobotAsset) {
  const bounds = new THREE.Box3().setFromObject(object);
  if (bounds.isEmpty()) {
    const target = robot.cameraTarget ?? [0.3, 0, 0.7];
    controls.target.set(target[0], target[1], target[2]);
    camera.position.set(2.4, -1.8, 1.9);
    camera.lookAt(controls.target);
    return;
  }

  const size = bounds.getSize(new THREE.Vector3());
  const center = bounds.getCenter(new THREE.Vector3());
  const target = robot.cameraTarget
    ? new THREE.Vector3(robot.cameraTarget[0], robot.cameraTarget[1], robot.cameraTarget[2])
    : center;

  const multiplier = Math.max(robot.cameraDistanceMultiplier ?? 1, 0.3);
  const radius = Math.max(size.length() * 0.45, 0.8) * multiplier;

  controls.target.copy(target);
  controls.minDistance = Math.max(radius * 0.4, 0.5);
  controls.maxDistance = Math.max(radius * 10, 10);
  camera.position.set(target.x + radius * 1.6, target.y - radius * 1.35, target.z + radius * 0.95);
  camera.near = Math.max(radius / 200, 0.01);
  camera.far = Math.max(radius * 30, 100);
  camera.updateProjectionMatrix();
  camera.lookAt(target);
}

function applyUrdfJointValues(joints: URDFJoint[], values: number[]) {
  for (let i = 0; i < joints.length; i += 1) {
    joints[i].setJointValue(values[i] ?? 0);
  }
}

export function SimulationView({ robot, trajectory = null, currentTimeSec = 0 }: SimulationViewProps) {
  const containerRef = React.useRef<HTMLDivElement | null>(null);
  const trajectoryRef = React.useRef<PlanningTrajectory | null>(trajectory);
  const currentTimeRef = React.useRef<number>(currentTimeSec);
  const [loadState, setLoadState] = React.useState<LoadState>('loading');
  const [statusText, setStatusText] = React.useState(`Loading ${robot.name}...`);
  const [errorText, setErrorText] = React.useState<string | null>(null);
  const [resolvedEngineLabel, setResolvedEngineLabel] = React.useState(robot.engineLabel);
  const [resolvedSourceType, setResolvedSourceType] = React.useState<RobotSourceType>(robot.sourceType);

  React.useEffect(() => {
    trajectoryRef.current = trajectory ?? null;
    if (loadState !== 'ready') return;
    setStatusText(trajectory?.tUniform?.length ? `Trajectory attached to ${robot.name}.` : `${robot.name} ready.`);
  }, [trajectory, robot.name, loadState]);

  React.useEffect(() => {
    currentTimeRef.current = currentTimeSec;
  }, [currentTimeSec]);

  React.useEffect(() => {
    let disposed = false;
    let animationFrame = 0;
    let viewport: ReturnType<typeof createViewport> | null = null;
    let model: MujocoModel | null = null;
    let data: MujocoData | null = null;
    let moduleInstance: MujocoModule | null = null;
    let urdfRobot: URDFRobot | null = null;
    let urdfJoints: URDFJoint[] = [];

    async function boot() {
      const container = containerRef.current;
      if (!container) return;

      try {
        setLoadState('loading');
        setErrorText(null);
        setStatusText(`Loading ${robot.name}...`);
        setResolvedEngineLabel(robot.engineLabel);
        setResolvedSourceType(robot.sourceType);

        const source = await resolveRobotSource(robot);
        if (disposed) return;

        setResolvedSourceType(source.sourceType);
        setResolvedEngineLabel(source.runtime === 'mujoco' ? 'MuJoCo WASM' : 'URDF Loader');

        viewport = createViewport(container);
        const displayRoot = new THREE.Group();
        applyDisplayTransform(displayRoot, robot);
        viewport.scene.add(displayRoot);

        const startMujoco = async () => {
          moduleInstance = await loadMujoco({
            printErr: (text: string) => {
              if (text && !disposed) {
                setErrorText(text);
              }
            },
          }) as unknown as MujocoModule;
          if (disposed) return;

          setStatusText(`Loading ${robot.name} MuJoCo assets...`);
          const modelPath = await populateVirtualFileSystem(moduleInstance, robot, source);
          if (disposed) return;

          setStatusText(`Compiling ${robot.name}...`);
          model = moduleInstance.MjModel.loadFromXML(modelPath);
          data = new moduleInstance.MjData(model);

          getDisplayedJointValues(robot, trajectoryRef.current, currentTimeRef.current).forEach((value, index) => {
            if (index < data!.qpos.length) {
              data!.qpos[index] = value;
            }
          });
          moduleInstance.mj_forward(model, data);

          const bodies: THREE.Group[] = [];
          for (let bodyIndex = 0; bodyIndex < model.nbody; bodyIndex += 1) {
            const group = new THREE.Group();
            bodies.push(group);
            displayRoot.add(group);
          }

          const profile = robot.materialProfile ?? 'neutral_metallic';
          for (let geomIndex = 0; geomIndex < model.ngeom; geomIndex += 1) {
            const bodyId = model.geom_bodyid[geomIndex];
            const geom = buildGeom(moduleInstance, model, geomIndex, profile);
            if (geom) {
              bodies[bodyId].add(geom);
            }
          }

          fitCameraToObject(viewport.camera, viewport.controls, displayRoot, robot);
          setLoadState('ready');
          setStatusText(`${robot.name} ready.`);

          const frameIntervalMs = 1000 / 30;
          let lastFrameTs = 0;
          const step = (ts: number) => {
            if (disposed || !viewport || !data || !model || !moduleInstance) return;
            if (ts - lastFrameTs < frameIntervalMs) {
              animationFrame = window.requestAnimationFrame(step);
              return;
            }
            lastFrameTs = ts;

            const sampledQ = getDisplayedJointValues(robot, trajectoryRef.current, currentTimeRef.current);
            const dof = Math.min(sampledQ.length, data.qpos.length);
            for (let i = 0; i < dof; i += 1) {
              data.qpos[i] = sampledQ[i];
            }
            moduleInstance.mj_forward(model, data);

            for (let bodyIndex = 0; bodyIndex < bodies.length; bodyIndex += 1) {
              const body = bodies[bodyIndex];
              body.position.set(data.xpos[bodyIndex * 3], data.xpos[bodyIndex * 3 + 1], data.xpos[bodyIndex * 3 + 2]);
              body.quaternion.set(
                data.xquat[bodyIndex * 4 + 1],
                data.xquat[bodyIndex * 4 + 2],
                data.xquat[bodyIndex * 4 + 3],
                data.xquat[bodyIndex * 4],
              );
            }

            viewport.controls.update();
            viewport.renderer.render(viewport.scene, viewport.camera);
            animationFrame = window.requestAnimationFrame(step);
          };

          animationFrame = window.requestAnimationFrame(step);
        };

        const startUrdf = async () => {
          if (source.sourceType !== 'URDF') {
            throw new Error(`URDF fallback unavailable for source type: ${source.sourceType}`);
          }

          setStatusText(`Loading ${robot.name} URDF...`);
          const loader = new URDFLoader();
          const sourceUrl = toAbsoluteAssetUrl(source.sourcePath);
          const sourceResponse = await fetch(sourceUrl);
          if (!sourceResponse.ok) {
            throw new Error(`Failed to load URDF source: ${source.sourcePath}`);
          }
          const sourceText = await sourceResponse.text();
          const normalizedUrdf = await normalizeUrdfPaths(robot, sourceText);
          loader.packages = toAbsoluteAssetUrl(robot.publicRoot).replace(/\/$/, '');
          loader.workingPath = toAbsoluteAssetUrl(`${getDirName(source.sourcePath)}/`);
          urdfRobot = loader.parse(normalizedUrdf.text);
          if (disposed || !viewport || !urdfRobot) return;

          applyUrdfVisualProfile(urdfRobot, robot.materialProfile ?? 'neutral_metallic');
          displayRoot.add(urdfRobot);
          fitCameraToObject(viewport.camera, viewport.controls, displayRoot, robot);

          urdfJoints = Object.values(urdfRobot.joints).filter((joint) => joint.jointType !== 'fixed');
          applyUrdfJointValues(urdfJoints, getDisplayedJointValues(robot, trajectoryRef.current, currentTimeRef.current));

          setLoadState('ready');
          setStatusText(`${robot.name} ready.`);

          const frameIntervalMs = 1000 / 30;
          let lastFrameTs = 0;
          const step = (ts: number) => {
            if (disposed || !viewport) return;
            if (ts - lastFrameTs < frameIntervalMs) {
              animationFrame = window.requestAnimationFrame(step);
              return;
            }
            lastFrameTs = ts;
            applyUrdfJointValues(urdfJoints, getDisplayedJointValues(robot, trajectoryRef.current, currentTimeRef.current));
            viewport.controls.update();
            viewport.renderer.render(viewport.scene, viewport.camera);
            animationFrame = window.requestAnimationFrame(step);
          };

          animationFrame = window.requestAnimationFrame(step);
        };

        if (source.runtime === 'mujoco') {
          try {
            await startMujoco();
            return;
          } catch (mujocoError) {
            if (source.sourceType !== 'URDF') {
              throw mujocoError;
            }
            console.warn('MuJoCo load failed, falling back to URDF loader.', mujocoError);
            setErrorText(null);
            setResolvedEngineLabel('URDF Loader (Fallback)');
          }
        }

        await startUrdf();
      } catch (error) {
        if (disposed) return;
        const message = error instanceof Error ? error.message : String(error);
        console.error('SimulationView init error', error);
        setLoadState('error');
        setErrorText(message);
        setStatusText(`Failed to load ${robot.name}.`);
      }
    }

    void boot();

    return () => {
      disposed = true;
      window.cancelAnimationFrame(animationFrame);
      data?.delete?.();
      model?.delete?.();
      try {
        moduleInstance?.FS?.unmount('/working');
      } catch {
        // Ignore teardown races.
      }
      urdfRobot?.removeFromParent();
      viewport?.dispose();
    };
  }, [robot]);

  const displayedQ = getDisplayedJointValues(robot, trajectory, currentTimeSec);

  return (
    <div className="relative h-full w-full overflow-hidden bg-[#ffffff]">
      <div ref={containerRef} className="absolute inset-0" />

      <div className="pointer-events-none absolute left-4 top-4 z-10 flex flex-col gap-1">
        <div className="flex items-center gap-2 border border-[#e5e5e5] bg-[#f3f3f3] px-2 py-0.5 text-[#333333]">
          <div className={`h-1.5 w-1.5 rounded-full ${loadState === 'error' ? 'bg-amber-500' : 'bg-emerald-500'}`} />
          <span className="text-[10px] font-bold uppercase tracking-wider">{`Engine: ${resolvedEngineLabel}`}</span>
        </div>
        <div className="flex items-center gap-2 border border-[#e5e5e5] bg-[#f3f3f3] px-2 py-0.5 text-[#333333]">
          <Activity className="h-3 w-3 text-blue-600" />
          <span className="text-[10px] font-medium">{statusText}</span>
        </div>
      </div>

      <div className="absolute right-4 top-4 z-10">
        <div className="w-44 border border-[#e5e5e5] bg-[#f3f3f3] p-2">
          <p className="mb-1.5 text-[9px] font-bold uppercase tracking-wider text-[#6f6f6f]">Joint States</p>
          <div className="space-y-1">
            {displayedQ.map((value, index) => {
              const progress = `${Math.min(100, Math.round((Math.abs(value) / Math.PI) * 100))}%`;
              const label = robot.jointLabels[index] ?? `J${index + 1}`;
              return (
                <div key={`joint-${index + 1}`} className="flex items-center gap-2">
                  <span className="w-7 text-[9px] font-mono text-[#6f6f6f]">{label}</span>
                  <div className="h-1 flex-1 overflow-hidden bg-[#e5e5e5]">
                    <div className="h-full bg-[#007acc]" style={{ width: progress }} />
                  </div>
                  <span className="text-[9px] font-mono text-[#333333]">{value.toFixed(2)}</span>
                </div>
              );
            })}
          </div>
        </div>
      </div>

      <div className="absolute bottom-4 left-4 z-10 flex gap-2">
        <div className="border border-[#e5e5e5] bg-[#f3f3f3] px-2 py-1">
          <span className="mr-2 text-[9px] font-bold uppercase tracking-wider text-[#6f6f6f]">Model</span>
          <span className="text-xs font-mono text-[#333333]">{robot.name}</span>
        </div>
        <div className="border border-[#e5e5e5] bg-[#f3f3f3] px-2 py-1">
          <span className="mr-2 text-[9px] font-bold uppercase tracking-wider text-[#6f6f6f]">Format</span>
          <span className="text-xs font-mono text-[#333333]">{resolvedSourceType}</span>
        </div>
      </div>

      {loadState !== 'ready' && (
        <div className="absolute inset-0 z-20 flex items-center justify-center bg-white/70 backdrop-blur-sm">
          <div className="rounded-xl border border-[#dbe4ee] bg-white px-5 py-4 text-center shadow-sm">
            <div className="text-sm font-semibold text-slate-800">
              {loadState === 'error' ? `${robot.name} load failed` : 'Loading simulation'}
            </div>
            <div className="mt-2 max-w-[320px] text-xs font-mono text-slate-500">{errorText ?? statusText}</div>
          </div>
        </div>
      )}
    </div>
  );
}
