#lzma_helper
Basically just slightly (and incompletely) refactored code from the cpp 7zip sdk.
Also added functionality to work with vectors.
The easiest usage is to just do:
    const auto uncompressed = lzma_decompress(compressed);
Might make this nicer in the future, so that it won't pollute the global namespace, for example.
