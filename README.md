# CuBaGa

CuBaGa is a tabletop game interchange format.

CuBaGa are webassembly files with a custom section for its resources (custom textures/meshes). The game logic is
exposed in the wasm code, it can react to some exports and call imports to spawn/move pieces or other i/o.

The input system is limited, to enforce making games accessible to player with different control hardware.

## Dependencies:

- [richgel999/bc7enc16](https://github.com/richgel999/bc7enc16)
- [jkuhlmann/cgltf](https://github.com/jkuhlmann/cgltf.git)
- [tcoppex/ext-mikktspace](https://github.com/tcoppex/ext-mikktspace)
- [Jiboo/libhut](https://github.com/Jiboo/libhut.git)
- [zeux/meshoptimizer](https://github.com/zeux/meshoptimizer.git)
- [nothings/stb](https://github.com/nothings/stb.git)

## File formats conventions

- cubaga.lz4: LZ4 compressed wasm with resources
- cubaga: uncompressed wasm with resources
- cbr: raw resources bundle
- wasm: webassembly without resources

## Tools

- Packager: converts a list of GLTF files into a CBR, usage: <output-file> <list of input files...>
- Viewer: allows to view a CBR file contents: <input-file> <material-id> <mesh-id>
- Debugger: allows to test a CUBAGA file

## Todo

- Packager:
    - Export multiple LOD using meshopt_simplify
    - Allows importing complex models (more than 64k vertices) by simplifying them first
- Viewer:
    - Allow to view textures too (and possibly select a channel)
    - IBL
- Debugger:
    - Dev not started yet

## Imports

Here's the list of functions that the webassembly module may import from the host, and use in reaction of events.

	using actor_ref = u64;
	using material_ref = u64;

	// aliases
	using piece_ref = actor_ref;
	using player_ref = actor_ref;

	enum action {
		NONE,
		OK, CANCEL,	EXTRA_0, EXTRA_1,
		UP, DOWN, LEFT, RIGHT,
		LOOK_AT,
	};

	void exit(u32 exit_code, char *message, u8 message_size);

	material_ref material_create(u8 material_id);
	void material_destroy(material_ref);
	void material_color_factor(u8[3] RGB_srgb);
	void material_emissive_factor(u8[3] RGBA_srgb);

    piece_ref piece_create(u8 mesh_id, material_ref);
    void piece_destroy(piece_ref);
    void piece_move(piece_ref, vec3 pos, qat4 rot);
    void piece_moveTo(piece_ref, vec3 pos, qat4 rot, float animTime);
    void piece_visibility(piece_ref, bool visible);
    void piece_material(piece_ref, material_ref);

    void camera_capture(player_ref, vec3 pos, qat4 rot);
    void camera_free(player_ref);

	void notify(player_ref, char *message_utf8, u8 message_size);
	void button_help(player_ref, button, char *message_utf8, u8 message_size);

## Exports

Here's the list of functions that the webassembly module may export to the host, so that it would get notified of
events (new frame, server error, ...).

	struct player_desc {
		player_ref id;
		char *name;
		u8 name_size;
		char[5] locale; // "en_US"
	}
	struct interaction {
		player_ref source;
		actor_ref target;
		action what;
	};

    int start(player_desc *players, u8 players_size);
    void tick(float delta_s, interaction *interactions, u32 interactions_size);

## Resources section

The resources bundle are stored in a webassembly custom section named "cubaga_res".

The format is loosely based on GLTF, as the pipeline was intended to take GLTF 2.0 models as input:
- Same coordinates system: right-handed, +Y as up, front of meshes face +Z, unit is meter,
- Only opaque static meshes,
- No nodes/scene, it's just a list of textures, materials and meshes,
- As it is supposed to hold "small" game objects, the vertex attributes elements are quantized (10/11bits), all
    textures are in BC7 and limited to 1024px,
- Textures extents must be a power of two,
- No samplers, engine may use whatever min/mag filters depending on required quality, and wrapping defaults to "repeat",
- Vertices and indices are encoded using meshoptimizer compression,
- Format is little-endian,

Vertex data consists of three U32:

	pos_x: 10_unorm,
	pos_y: 11_unorm,
	pos_z: 11_unorm,

	nor_x: 8_snorm,
	nor_y: 8_snorm,
	nor_z: 8_snorm,
	tan_sign: 1_unorm,
	tan_x: 7_snorm,

	tan_y: 6_snorm,
	tan_z: 6_snorm,
	tex_u: 10_unorm,
	tex_v: 10_unorm,

Here's the file format description:

	u32 magic (must be "cbg1")
	u8 textures_count (limited to 254)
	for each texture:
		u4 xpow (limited to 10, 2^xpow = width at mip level 0)
		u4 ypow (limited to 10, 2^ypow = height at mip level 0)
		uleb128 byte_size
		uleb128 byte_offset (encoded in BC7 RGB at 1 byte/pixel, all possible mip levels contiguous)
	u8 materials_count
	for each material:
		u8 albedo_tex_id (BC7_SRGB, index or 0xff if ommited, defaults to white)
		u8 emissive_tex_id (BC7_SRGB, index or 0xff if ommited, defaults to black)
		u8 normals_tex_id (BC7_UNORM, index or 0xff if ommited, defaults to grey?)
		u8 orm_tex_id (BC7_UNORM R=occlusion G=roughness B=metallic, index or 0xff if ommited, defaults to white)
		u8[3] color_factor (R8G8B8 SRGB, either scales the albedo tex fetched value, or used as base color if no albedo texture)
		u8 roughness_factor (either scales the roughness tex fetched value, or used as base roughness if no ORM texture)
		u8[3] emissive_factor (R8G8B8 SRGB, either scales the emissive tex fetched value, or used as base emissive color if no emissive texture)
		u8 metallic_factor (either scales the metallic tex fetched value, or used as base metallic if no ORM texture)
	u8 mesh_count
	for each mesh:
		float[3] translate
		float[3] scale (you may guess an AABB from scale/translate as they will be calculated to make all positions axis in the range [0; 1])
		u16 vertices_count
		uleb128 encoded_vertices_byte_size
		uleb128 encoded_vertices_byte_offset
		u8 lod_levels (must be at least 1, max 5)
		for each lod_level:
			uleb128 indices_count (if lod_level>0, must be at least half of lod_level-1)
			uleb128 encoded_indices_byte_size
			uleb128 encoded_indices_byte_offset
	uleb128 data_size
	u8[data_size] data
