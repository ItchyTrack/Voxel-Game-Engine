import init, {
	voxel_sources_chunk_worker_loop,
	voxel_sources_lod_worker_loop,
} from "../../../pkg/bevy_web_worker.js";

self.onmessage = async (event) => {
	const data = event.data;
	if (!data || typeof data !== 'object' || !('module' in data) || !('memory' in data) || !('kind' in data) || !('workerId' in data)) {
		throw new Error('source request worker received invalid init payload');
	}
	await init({ module_or_path: data.module, memory: data.memory });
	if (data.kind === 'chunk') {
		voxel_sources_chunk_worker_loop(data.workerId);
	} else if (data.kind === 'lod') {
		voxel_sources_lod_worker_loop(data.workerId);
	} else {
		throw new Error(`unknown source request worker kind: ${data.kind}`);
	}
};
