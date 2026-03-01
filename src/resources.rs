// use std::io::{BufReader, Cursor};
// use wgpu::util::DeviceExt;
// use crate::gpu_objects::{matrix, mesh};

// #[cfg(target_arch = "wasm32")]
// fn format_url(file_name: &str) -> reqwest::Url {
// 	let window = web_sys::window().unwrap();
// 	let location = window.location();
// 	let mut origin = location.origin().unwrap();
// 	if !origin.ends_with("res") {
// 		origin = format!("{}/res", origin);
// 	}
// 	let base = reqwest::Url::parse(&format!("{}/", origin,)).unwrap();
// 	base.join(file_name).unwrap()
// }

// pub async fn load_string(file_name: &str) -> anyhow::Result<String> {
// 	#[cfg(target_arch = "wasm32")]
// 	let txt = {
// 		let url = format_url(file_name);
// 		reqwest::get(url).await?.text().await?
// 	};
// 	#[cfg(not(target_arch = "wasm32"))]
// 	let txt = {
// 		let path = std::path::Path::new(env!("OUT_DIR"))
// 			.join("res")
// 			.join(file_name);
// 		std::fs::read_to_string(path)?
// 	};

// 	Ok(txt)
// }

// pub async fn load_binary(file_name: &str) -> anyhow::Result<Vec<u8>> {
// 	#[cfg(target_arch = "wasm32")]
// 	let data = {
// 		let url = format_url(file_name);
// 		reqwest::get(url).await?.bytes().await?.to_vec()
// 	};
// 	#[cfg(not(target_arch = "wasm32"))]
// 	let data = {
// 		let path = std::path::Path::new(env!("OUT_DIR"))
// 			.join("res")
// 			.join(file_name);
// 		std::fs::read(path)?
// 	};

// 	Ok(data)
// }

// // pub async fn load_texture(
// //     file_name: &str,
// //     device: &wgpu::Device,
// //     queue: &wgpu::Queue,
// // ) -> anyhow::Result<texture::Texture> {
// //     let data = load_binary(file_name).await?;
// //     texture::Texture::from_bytes(device, queue, &data, file_name)
// // }

// pub async fn load_model(
// 	file_name: &str,
// 	device: &wgpu::Device,
// ) -> anyhow::Result<mesh::Mesh> {
// 	let obj_text = load_string(file_name).await?;
// 	let obj_cursor = Cursor::new(obj_text);
// 	let mut obj_reader = BufReader::new(obj_cursor);

// 	let (models, _obj_materials) = tobj::load_obj_buf_async(
// 		&mut obj_reader,
// 		&tobj::LoadOptions {
// 			triangulate: true,
// 			single_index: true,
// 			..Default::default()
// 		},
// 		|p| async move {
// 			let mat_text = load_string(&p).await.unwrap();
// 			tobj::load_mtl_buf(&mut BufReader::new(Cursor::new(mat_text)))
// 		},
// 	)
// 	.await?;

// 	let meshes = models
// 		.into_iter()
// 		.map(|m| {
// 				let vertices = (0..m.mesh.positions.len() / 3)
// 				.map(|i| {
// 					if m.mesh.normals.is_empty(){
// 						mesh::MeshVertex {
// 							position: [
// 								m.mesh.positions[i * 3],
// 								m.mesh.positions[i * 3 + 1],
// 								m.mesh.positions[i * 3 + 2],
// 							],
// 							color: [
// 								m.mesh.texcoords[i * 2],
// 								1.0 - m.mesh.texcoords[i * 2 + 1],
// 								0.0,
// 								1.0
// 							],
// 							normal: [0.0, 0.0, 0.0],
// 						}
// 					}else{
// 						mesh::MeshVertex {
// 							position: [
// 								m.mesh.positions[i * 3],
// 								m.mesh.positions[i * 3 + 1],
// 								m.mesh.positions[i * 3 + 2],
// 							],
// 							color: [
// 								m.mesh.texcoords[i * 2],
// 								1.0 - m.mesh.texcoords[i * 2 + 1],
// 								0.0,
// 								1.0
// 							],
// 							normal: [
// 								m.mesh.normals[i * 3],
// 								m.mesh.normals[i * 3 + 1],
// 								m.mesh.normals[i * 3 + 2],
// 							],
// 						}
// 					}
// 				})
// 				.collect::<Vec<_>>();

// 			let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
// 				label: Some(&format!("{:?} Vertex Buffer", file_name)),
// 				contents: bytemuck::cast_slice(&vertices),
// 				usage: wgpu::BufferUsages::VERTEX,
// 			});
// 			let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
// 				label: Some(&format!("{:?} Index Buffer", file_name)),
// 				contents: bytemuck::cast_slice(&m.mesh.indices),
// 				usage: wgpu::BufferUsages::INDEX,
// 			});

// 			let matrix_buffer = matrix::MatrixUniform::get_buffer(device, 0);
// 			mesh::Mesh {
// 				vertex_buffer,
// 				index_buffer,
// 				num_elements: m.mesh.indices.len() as u32,
// 				matrix_buffer: matrix_buffer.0,
// 				matrix_bind_group: matrix_buffer.1,
// 			}
// 		})
// 		.collect::<Vec<_>>();

// 	Ok( meshes.into_iter().next().unwrap() )
// }
