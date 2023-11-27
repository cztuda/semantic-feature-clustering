function isOct = isOctave()
    isOct = exist('OCTAVE_VERSION', 'builtin') ~= 0;
end