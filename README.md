# CuBaGa

CuBaGa is a tabletop game interchange format.

CuBaGa are webassembly files with a custom section for its resources (custom textures/meshes). The game logic is
exposed in the wasm code, it can react to some exports and call imports to spawn/move pieces or other i/o.

The input system is limited, to enforce making games accessible to player with different control hardware.

## Dependencies:

- [Jiboo/libhut](https://github.com/Jiboo/libhut)
- [richgel999/bc7enc_rdo](https://github.com/richgel999/bc7enc_rdo)
- [jkuhlmann/cgltf](https://github.com/jkuhlmann/cgltf)
- [tcoppex/ext-mikktspace](https://github.com/tcoppex/ext-mikktspace)
- [zeux/meshoptimizer](https://github.com/zeux/meshoptimizer)
- [nothings/stb](https://github.com/nothings/stb)

Rendering pipeline heavily inspired from [SaschaWillems/Vulkan-glTF-PBR](https://github.com/KhronosGroup/SaschaWillems/Vulkan-glTF-PBR).
This repo contains some envmaps from [KhronosGroup/glTF-Sample-Environments](https://github.com/KhronosGroup/glTF-Sample-Environments)
and some models from [KhronosGroup/glTF-Sample-Models](https://github.com/KhronosGroup/glTF-Sample-Models).

## File formats conventions

- .cubaga.lz4: LZ4 compressed wasm with resources
- .cubaga: uncompressed wasm with resources
- .crb: raw resources bundle
- .wasm: webassembly without resources

## Tools

- Packager: converts a list of GLTF files into a CBR
- Viewer: allows to view a CBR file contents
- Debugger: allows to test a CUBAGA file
- PbrGen: generate IBL envmaps for viewer/debugger.

## Todo

- Format:
  - Possible move to [BinomialLLC/basis_universal](https://github.com/BinomialLLC/basis_universal) UASTC tex format for portability (but will need uastc=>bc7 transcoding)
  - "Dependencies", wasm-like imports but for textures/meshes from another CRB (and make it "safe" enough so that it's not used to bypass the limits on number of meshes/textures)
- Packager:
  - Export multiple LOD using meshopt_simplify
  - Allow importing complex models (more than 64k vertices) by simplifying them first
- PbrGen:
  - The result is not on par with what Vulkan-glTF-PBR outputs
- Viewer:
  - Currently, show one mesh+material from CLI, refactor to make it real browser of CRB files (view textures, any model, etc)
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
        OK, CANCEL,    EXTRA_0, EXTRA_1,
        UP, DOWN, LEFT, RIGHT,
        LOOK_AT,
    };

    void exit(u32 exit_code, char *message, u8 message_size);

    piece_ref piece_create(u32 mesh_id, u32 material_id);
    void piece_destroy(piece_ref);
    void piece_move(piece_ref, vec3 pos, qat4 rot);
    void piece_visibility(piece_ref, bool visible);
    void piece_color_factor(material_ref, u8[3] RGB_srgb);
    void piece_emissive_factor(material_ref, u8[3] RGB_srgb);
    void piece_roughness_factor(material_ref, float factor);
    void piece_metallic_factor(material_ref, float factor);

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
- As it is supposed to hold "small" objects, vertex attributes are quantized, and textures are limited to 1024px,
- Textures must be squares, a power of two size, and compressed as BC7,
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
    u32 textures_count (limited to 254)
    for each texture:
        u8 xpow (limited to 10, 2^xpow = width at mip level 0)
        u8 ypow (limited to 10, 2^xpow = width at mip level 0)
        u8 format
            0 => BC7_SRGB (albedo/emissive)
            1 => BC7_UNORM (orm/normals)
            2 => R8G8B8A8_SRGB
            3 => R8G8B8A8_UNORM
        u8 reserved
        for each possible mip_level (no mip under size of 4 pixels in width and/or height)
            u32 byte_offset
            u32 byte_size

    u32 materials_count (limited to 255)
    for each material:
        u8 albedo_tex_id (index or 0xff if ommited, defaults to white)
        u8 emissive_tex_id (index or 0xff if ommited, defaults to black)
        u8 normals_tex_id (index or 0xff if ommited, defaults to "flat")
        u8 orm_tex_id (R=occlusion G=roughness B=metallic, index or 0xff if ommited, defaults to white)

        u8[3] color_factor (R8G8B8 sRGB, either scales the albedo tex fetched value, or used as base color if no albedo texture)
        u8 roughness_factor (either scales the roughness tex fetched value, or used as base roughness if no ORM texture)
        u8[3] emissive_factor (R8G8B8 sRGB, either scales the emissive tex fetched value, or used as base emissive color if no emissive texture)
        u8 metallic_factor (either scales the metallic tex fetched value, or used as base metallic if no ORM texture)

    u32 mesh_count (limited to 255)
    for each mesh:
        float[3] translate
        float[3] scale (all vertices positions are normalized, use these transforms to render them normally)
        u32 vertices_count (each vertex is 3 u32, max vertices count is 64k)
        u32 encoded_vertices_byte_offset (vertices are encoded using meshoptimizer compression)
        u32 encoded_vertices_byte_size

        u32 lod_levels
        for each lod_level:
            u32 indices_count (if lod_level>0, must be at least half of previous lod_level)
            u32 encoded_indices_byte_offset (indices are encoded using meshoptimizer compression)
            u32 encoded_indices_byte_size

    u32 data_size
    u8[data_size] data
