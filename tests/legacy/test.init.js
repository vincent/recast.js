import { createRequire } from 'module';

const require = createRequire(import.meta.url);
const Recast = require('../lib/recast.js');

console.log(Recast)

let recast = await Recast();

console.log(recast)
