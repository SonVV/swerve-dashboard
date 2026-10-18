import asyncio, json, math, os, time
from pathlib import Path
from aiohttp import web, WSMsgType

ROOT=Path(__file__).parent
clients=set()
state={'x':0.,'y':0.,'heading':0.,'vx':0.,'vy':0.,'omega':0.,'enabled':False}
command={'vx':0.,'vy':0.,'omega':0.,'enabled':False,'heading_target':0.,'heading_mode':False}
last_command=0.
WHEEL_POS=[(.28,.22),(.28,-.22),(-.28,.22),(-.28,-.22)]
previous=[0.]*4

def clamp(x, lo, hi): return max(lo,min(hi,x))

async def index(request): return web.FileResponse(ROOT/'index.html')
async def status(request): return web.json_response({'status':'ok','clients':len(clients),'mode':'simulation'})

async def ws(request):
    global last_command
    socket=web.WebSocketResponse(heartbeat=10,max_msg_size=2048)
    await socket.prepare(request)
    clients.add(socket)
    try:
        async for msg in socket:
            if msg.type!=WSMsgType.TEXT: continue
            try:
                data=json.loads(msg.data)
                if data.get('type')!='command': continue
                command.update(vx=clamp(float(data.get('vx',0)),-1,1),vy=clamp(float(data.get('vy',0)),-1,1),omega=clamp(float(data.get('omega',0)),-2,2),enabled=data.get('enabled') is True,heading_target=clamp(float(data.get('heading_target',0)),-180,180),heading_mode=data.get('heading_mode') is True)
                last_command=time.monotonic()
            except (ValueError,TypeError,OverflowError,KeyError): continue
    finally:
        clients.discard(socket)
        command.update(vx=0.,vy=0.,omega=0.,enabled=False)
    return socket

async def simulator(app):
    global previous
    tick=0.05
    while True:
        start=time.monotonic()
        active=command['enabled'] and (start-last_command)<0.5
        for k in ('vx','vy','omega'):
            target=(clamp(2.5*((math.radians(command['heading_target'])-state['heading']+math.pi)%(2*math.pi)-math.pi),-2,2) if k=='omega' and command['heading_mode'] else command[k]) if active else 0.
            state[k]+=clamp(target-state[k],-2*tick,2*tick)
        state['enabled']=active
        h=state['heading']
        state['x']+=(state['vx']*math.cos(h)-state['vy']*math.sin(h))*tick
        state['y']+=(state['vx']*math.sin(h)+state['vy']*math.cos(h))*tick
        state['heading']=(h+state['omega']*tick+math.pi)%(2*math.pi)-math.pi
        wheels=[]
        for i,(x,y) in enumerate(WHEEL_POS):
            wx=state['vx']-state['omega']*y
            wy=state['vy']+state['omega']*x
            speed=math.hypot(wx,wy)
            angle=math.degrees(math.atan2(wy,wx)) if speed>0.001 else 0
            acceleration=(speed-previous[i])/tick
            previous[i]=speed
            wheels.append({'id':['FL','FR','RL','RR'][i],'speed':round(speed,3),'accel':round(acceleration,3),'steering':round(angle,1)})
        packet=json.dumps({'type':'telemetry','t':round(time.time(),3),'pose':{'x':round(state['x'],3),'y':round(state['y'],3),'heading':round(math.degrees(state['heading']),1)},'wheels':wheels,'enabled':active},allow_nan=False)
        if clients:
            await asyncio.gather(*(c.send_str(packet) for c in tuple(clients)),return_exceptions=True)
        await asyncio.sleep(max(0,tick-(time.monotonic()-start)))

async def background(app):
    task=asyncio.create_task(simulator(app))
    yield
    task.cancel()
    await asyncio.gather(task,return_exceptions=True)

app=web.Application()
app.router.add_get('/',index)
app.router.add_get('/health',status)
app.router.add_get('/ws',ws)
app.cleanup_ctx.append(background)
if __name__=='__main__':
    web.run_app(app,host=os.getenv('HOST','0.0.0.0'),port=int(os.getenv('PORT','8080')),print=lambda x:print(x,flush=True))
