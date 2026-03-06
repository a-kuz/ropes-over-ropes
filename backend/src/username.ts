const adjectives = [
  "Swift", "Brave", "Clever", "Lucky", "Bold", "Quick", "Calm", "Keen",
  "Wise", "Bright", "Sharp", "Sly", "Warm", "Cool", "Wild", "Fierce",
  "Noble", "Proud", "Agile", "Vivid", "Deft", "Witty", "Jolly", "Zesty",
  "Sleek", "Plucky", "Spry", "Nimble", "Merry", "Grand", "Stout", "Snappy",
  "Daring", "Crafty", "Lively", "Gentle", "Hardy", "Mighty", "Steady", "Eager",
  "Peppy", "Frisky", "Perky", "Chipper", "Zippy", "Breezy", "Sunny", "Rustic",
];

const animals = [
  "Fox", "Bear", "Wolf", "Hawk", "Lynx", "Otter", "Raven", "Eagle",
  "Tiger", "Panda", "Koala", "Falcon", "Heron", "Crane", "Viper", "Gecko",
  "Moose", "Bison", "Whale", "Shark", "Robin", "Finch", "Owl", "Badger",
  "Marten", "Ferret", "Hare", "Stag", "Seal", "Puma", "Cobra", "Ibis",
  "Wren", "Dove", "Jay", "Lark", "Swan", "Newt", "Toad", "Crab",
  "Moth", "Wasp", "Ant", "Bee", "Elk", "Ram", "Yak", "Emu",
];

export function generateUsername(): string {
  const adj = adjectives[Math.floor(Math.random() * adjectives.length)];
  const animal = animals[Math.floor(Math.random() * animals.length)];
  const num = Math.floor(Math.random() * 100);
  return `${adj}${animal}${num}`;
}
