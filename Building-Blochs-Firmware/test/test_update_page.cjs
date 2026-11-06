const assert = require('node:assert/strict');
const {readFileSync} = require('node:fs');
const vm = require('node:vm');
const {webcrypto} = require('node:crypto');
const {test} = require('node:test');
const script = readFileSync(require('node:path').join(__dirname,'../update.html'),'utf8').match(/<script type="module">([\s\S]*?)<\/script>/)[1];
function setup({legacy=false, failure=0, disconnect=false, cancel=false}={}) {
  const elements = {};
  const $ = id => elements[id] ||= {value:'',textContent:'',disabled:false,checked:false,removeAttribute(name){delete this[name];}};
  const packets=[], listeners={};
  let code=0,offset=0,staged=[],download=false,reads=0;
  const device={name:'Bloch-1',gatt:{connected:true,disconnect(){this.connected=false;listeners.gattserverdisconnected?.();},async connect(){this.connected=true;return {getPrimaryService:async()=>service};}},addEventListener(name,fn){listeners[name]=fn;}};
  const writer={async writeValueWithResponse(packet){
    packets.push([...packet]);
    const op=packet[0];
    if(op===0x10){staged=[];offset=0;code=8;}
    else if(op===0x11){assert.equal(packet[1]+256*packet[2],staged.length);staged.push(...packet.slice(3));offset=staged.length;code=8;}
    else if(op===0x12)code=9;
    else if(op===0x13){download=true;offset=0;code=1;}
    else if(op===3){code=0;download=false;}
    else if(op===1){offset=0;code=1;}
    else if(op===2){assert.equal(new DataView(packet.buffer).getUint32(1,true),offset);offset+=packet.length-5;code=1;}
    else if(op===4)code=2;
    else throw Error('Unknown opcode');
  }};
  const status={async readValue(){
    if(download&&++reads>1){
      if(disconnect){device.gatt.disconnect();throw Error('Disconnected');}
      if(cancel&&reads===2)$('cancel').onclick();
      code=failure|| (cancel?1:2);offset=512;
    }
    const reply=new DataView(new ArrayBuffer(5));reply.setUint8(0,code);reply.setUint32(1,offset,true);return reply;
  }};
  const service={async getCharacteristic(uuid){
    const n=uuid[7];
    if(n==='1')return writer;
    if(n==='2')return status;
    if(n==='3')return {readValue:async()=>new TextEncoder().encode('CoreS3;test')};
    if(n==='4'&&!legacy)return {readValue:async()=>new DataView(Uint8Array.of(3).buffer)};
    throw Error('Not supported');
  }};
  const ctx=vm.createContext({document:{getElementById:$},navigator:{bluetooth:{requestDevice:async()=>device}},TextEncoder,TextDecoder,Uint8Array,DataView,ArrayBuffer,URL,crypto:webcrypto,setTimeout:fn=>fn(),setInterval:()=>0});
  vm.runInContext(script,ctx);
  return {$,packets,staged:()=>staged,ctx};
}
test('saves UTF-8 credentials using minimum-MTU fragments and clears password',async()=>{
  const app=setup();await app.$('connect').onclick();
  app.$('ssid').value='Café, downstairs';app.$('password').value='secret,12345';
  await app.$('saveWifi').onclick();
  const ssid=new TextEncoder().encode('Café, downstairs');
  assert.deepEqual(app.staged(),[ssid.length,...ssid,...new TextEncoder().encode('secret,12345')]);
  assert(app.packets.every(p=>p.length<=20));assert.equal(app.$('password').value,'');
  assert.match(app.$('status').textContent,/details saved/);
});
test('rejects overlong UTF-8 SSID before sending',async()=>{
  const app=setup();await app.$('connect').onclick();app.$('ssid').value='é'.repeat(17);
  await app.$('saveWifi').onclick();assert.equal(app.packets.length,0);assert.match(app.$('status').textContent,/1–32/);
});
test('downloads direct URL and reports commit',async()=>{
  const app=setup();await app.$('connect').onclick();app.$('url').value='https://example.com/releases/update.bbu?token=abc';
  await app.$('wifiUpdate').onclick();
  assert.equal(new TextDecoder().decode(Uint8Array.from(app.staged())),app.$('url').value);
  assert.equal(app.packets.at(-1)[0],0x13);assert.match(app.$('status').textContent,/Update installed/);
});
test('surfaces network failures and cancels the session',async()=>{
  const app=setup({failure:11});await app.$('connect').onclick();app.$('url').value='https://example.com/update.bbu';
  await app.$('wifiUpdate').onclick();assert.equal(app.packets.at(-1)[0],3);assert.match(app.$('status').textContent,/Download failed/);
});
test('Bluetooth loss does not claim a Wi-Fi update was aborted',async()=>{
  const app=setup({disconnect:true});await app.$('connect').onclick();app.$('url').value='https://example.com/update.bbu';
  await app.$('wifiUpdate').onclick();assert.equal(app.packets.at(-1)[0],0x13);assert.match(app.$('status').textContent,/may still be running/);
});
test('cancel requests abort over Bluetooth',async()=>{
  const app=setup({cancel:true});await app.$('connect').onclick();app.$('url').value='https://example.com/update.bbu';
  await app.$('wifiUpdate').onclick();assert.equal(app.packets.at(-1)[0],3);assert.match(app.$('status').textContent,/Cancelled/);
});
test('old firmware keeps Bluetooth update available and disables Wi-Fi controls',async()=>{
  const app=setup({legacy:true});await app.$('connect').onclick();
  assert.equal(app.$('saveWifi').disabled,true);assert.equal(app.$('wifiUpdate').disabled,true);
  vm.runInContext('bytes=new Uint8Array(512);controls();',app.ctx);
  assert.equal(app.$('update').disabled,false);
  await app.$('update').onclick();assert.equal(app.packets.at(-1)[0],4);assert.match(app.$('status').textContent,/Update installed/);
  const data=app.packets.filter(p=>p[0]===2);assert.equal(data.reduce((n,p)=>n+p.length-5,0),512);
});
test('invalid schemes and credential-bearing URLs never reach the device',async()=>{
  for(const url of ['ftp://example.com/a.bbu','https://user:pass@example.com/a.bbu']){
    const app=setup();await app.$('connect').onclick();app.$('url').value=url;
    await app.$('wifiUpdate').onclick();assert(!app.packets.some(p=>p[0]===0x13));assert.match(app.$('status').textContent,/direct HTTP/);
  }
});
