import init, {
	voxel_sources_request_worker_loop,
} from "../../../pkg/bevy_web_worker.js";

self.onmessage = async (event) => {
	const data = event.data;
	if (!data || typeof data !== 'object' || !('module' in data) || !('memory' in data) || !('workerId' in data)) {
		throw new Error('source request worker received invalid init payload');
	}
	await init({ module_or_path: data.module, memory: data.memory });
	voxel_sources_request_worker_loop(data.workerId);
};
