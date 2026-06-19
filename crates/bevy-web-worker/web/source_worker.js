import init, { voxel_data_child_entry_point } from "../../../pkg/bevy_web_worker.js";

let ready = null;
let entryName = null;

self.onmessage = async (event) => {
	const data = event.data;
	if (data && typeof data === 'object' && 'module' in data && 'memory' in data && 'entryName' in data) {
		ready = init({ module_or_path: data.module, memory: data.memory });
		entryName = data.entryName;
		await ready;
		return;
	}

	if (!ready || !entryName) {
		throw new Error("source worker received job before init");
	}
	try {
		await ready;
		if (entryName === 'voxel_data_child_entry_point') {
			await voxel_data_child_entry_point(data);
		} else {
			throw new Error(`unknown worker entry: ${entryName}`);
		}
		self.postMessage({ ptr: data });
	} catch (e) {
		console.error(`source worker job failed entry=${entryName}`, e);
		self.postMessage({ type: 'job_error', error: String(e), ptr: data });
	}
};
