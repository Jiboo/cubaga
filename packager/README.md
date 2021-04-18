Packager used to convert a bundle of GLTF files into a CRB (cubaga resources bundle).

Usage: `cubaga-packager <output crb file path> <gltf input paths...>`

Main phases:
- Import: load "raws" from the GLTF (unencoded/uncompressed data)
- Process: compress, generate mips, lods, encode, ...
- Layout: attribute ranges in the data blob
- Dump: write the CRB
