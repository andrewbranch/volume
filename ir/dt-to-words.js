const down = [0.00900,0.00448,0.00060,0.00056,0.00056,0.00164,0.00060,0.00056,0.00056,0.00056,0.00056,0.00056,0.00056,0.00056,0.00056,0.00056,0.00056,0.00056,0.00056,0.00168,0.00056,0.00056,0.00056,0.00168,0.00056,0.00168,0.00056,0.00164,0.00060,0.00168,0.00056,0.00168,0.00060,0.00056,0.00052,0.00168,0.00060,0.00056,0.00052,0.00168,0.00060,0.00164,0.00060,0.00056,0.00056,0.00056,0.00056,0.00164,0.00060,0.00056,0.00056,0.00056,0.00056,0.00168,0.00056,0.00052,0.00060,0.00056,0.00056,0.00164,0.00060,0.00168,0.00056,0.00052,0.00060,0.00168,0.00056,0.04092,0.00900,0.00220,0.00060];
const up = [0.00898, 0.00446, 0.00060, 0.00054, 0.00058, 0.00164, 0.00060, 0.00052, 0.00060, 0.00056, 0.00054, 0.00056, 0.00058, 0.00056, 0.00056, 0.00056, 0.00056, 0.00056, 0.00056, 0.00166, 0.00058, 0.00052, 0.00060, 0.00166, 0.00058, 0.00166, 0.00060, 0.00164, 0.00060, 0.00166, 0.00058, 0.00166, 0.00058, 0.00056, 0.00056, 0.00056, 0.00056, 0.00056, 0.00056, 0.00058, 0.00054, 0.00166, 0.00060, 0.00054, 0.00056, 0.00056, 0.00058, 0.00166, 0.00058, 0.00052, 0.00060, 0.00166, 0.00056, 0.00168, 0.00058, 0.00164, 0.00060, 0.00054, 0.00060, 0.00164, 0.00060, 0.00166, 0.00056, 0.00054, 0.00060, 0.00166, 0.00056, 0.04092, 0.00900, 0.00222, 0.00060];

const hex = up
  .slice(0, 66)
  .map(x => Math.round(x * 100000 / 56))
  .reduce((acc, val, index) => {
    return acc + Array.apply([], Array(val)).map(() => Number(!(index % 2))).join('');
  }, '').split('').reduce((words, bit, index) => {
    if (index % 16 === 0) {
      return [...words, bit];
    }
    words[words.length - 1] = words[words.length - 1] + bit;
    return words;
  }, []).map(word => {
    const missingBits = Array.apply([], Array(16 - word.length)).map(() => '0').join('');
    const hexWord = parseInt(word + missingBits, 2).toString(16);
    const missingNybbles = Array.apply([], Array(4 - hexWord.length)).map(() => '0').join('');
    return `0x${missingNybbles}${hexWord.toUpperCase()}`;
  });

console.log(hex);