import init, { BevyApp } from "../../../pkg/bevy_web_worker.js";

let initPromise = null;
let app = null;

function ensureInit() {
	if (!initPromise) initPromise = init();
	return initPromise;
}

function frame() {
	if (!app) return;
	app.update();
	self.requestAnimationFrame(frame);
}

self.onmessage = async (event) => {
	const data = event.data;
	switch (data?.type) {
		case "init": {
			await ensureInit();
			const shimUrl = new URL("../../../pkg/bevy_web_worker.js", import.meta.url).href;
			app = new BevyApp(data.canvas, data.width, data.height, shimUrl);
			self.requestAnimationFrame(frame);
			self.postMessage({ type: "ready" });
			break;
		}
		case "resize": {
			app?.resize(data.width, data.height);
			break;
		}
		case "key": {
			app?.key(data.code, data.pressed);
			break;
		}
		case "mouse_button": {
			app?.mouse_button(data.button, data.pressed);
			break;
		}
		case "mouse_motion": {
			app?.mouse_motion(data.deltaX, data.deltaY);
			break;
		}
	}
};
