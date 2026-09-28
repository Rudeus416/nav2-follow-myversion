#!/usr/bin/env python3
"""Offline browser regression: real assets, localhost mocks, no robot requests.
R28: two-axis map panning uses real mouse/touch input; verify render alignment,
world-to-grid selection, brush coordinates, and read-only view controls.
R31: a valid locked person route remains executable when live observations
are missing/stale; no automatic replanning, and backend rejections still gate.
Person navigation: preserve explicit request errors across polling; show and
respect single/continuous start readiness, including a missing-field upgrade notice.
R27: verify that only the self-owned navigation panel is reorganized; preserve
the main page order and all original camera/manual/control nodes outside it. GETs serve deterministic data; every POST is recorded only in RAM.
The CDP request allowlist rejects any origin except the new localhost mock server.
This validates browser wiring/layout, not Nav2 planning or physical robot safety.

Run from the repository with the follow_demo Python environment::

    python -B nav2/tests/check_frontend_layout.py

Requires requests, websocket-client, and the locally installed Chromium snap.
Screenshots/report default to /tmp/nav2-layout-check; no project runtime data is
written. The Chromium process and its private temporary profile are cleaned up.
"""
import argparse, base64, json, os, re, shutil, socket, subprocess, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit
import requests
import websocket

ROOT=Path(__file__).resolve().parents[2]
IMAGE=b'''<svg xmlns="http://www.w3.org/2000/svg" width="960" height="540" viewBox="0 0 960 540"><rect width="960" height="540" fill="#18324d"/><path d="M0 540L320 220H640L960 540" fill="#587385"/><path d="M320 220V0M640 220V0M480 220V540" stroke="#98cbd6" stroke-width="3"/><rect x="390" y="100" width="100" height="260" rx="8" fill="none" stroke="#ffb545" stroke-width="5"/><text x="36" y="55" fill="white" font-size="28">OFFLINE CAMERA / ID 17</text><text x="36" y="500" fill="white" font-size="23">MOCK DATA - NO ROBOT CONNECTION</text></svg>'''

class State:
    def __init__(self):
        self.lock=threading.RLock();self.posts=[];self.gets=[];self.stopped=True
        self.fail_map=False;self.revision=0;self.rectangles=[];self.preview=None;self.vision=True
        self.person_preview_error=None;self.person_execute_error=None;self.start_readiness='auto'
        self.people_override=None;self.preview_sequence=1;self.target_marker=None
    def status(self):
        keys=set(re.findall(r'\bs\.([A-Za-z_]\w*)',(ROOT/'frontend/app.js').read_text()))
        out=dict.fromkeys(keys,0)
        out.update(nav2_enabled=True,nav2_obstacle_mode='map',force_stopped=self.stopped,
            motion_mode='auto',following=False,follow_settings=self.follow(),radar_settings={'enabled':False},
            processed_image_age_seconds=.1,image_stream_active=True,device_mode='managed',
            camera_process_running=True,device_control_available=True,visible_people=[{'id':17}],
            radar_to_camera_optical_4x4=[],nav2_vision_enabled=self.vision,nav2_vision_age=.2,
            nav2_vision_points=75,nav2_vision_error='',target_visible=True,target_distance_m=3.,
            nav2_preview=self.preview or {'state':'idle','message':'等待规划','points':[],'age':0},
            nav2_error='',nav2_last_stop_reason='',motion_reason='离线强停模拟' if self.stopped else '离线运行模拟',
            error='',write_error='',processing_error='',radar_error='',
            preview_target={'target_id':17,'visible_in_latest_tracking':True,'last_distance_m':3.,
                'tracking_age':.1,'measurement_age':.2,'timeout':2.,'ready':True,'problems':[]})
        return out
    def follow(self):
        return dict(camera_height_m=.5,follow_distance_m=1.,target_id=17,distance_mode='mask-depth',
            process_fps=10,depth_fps=5,persistent_identity_enabled=False)
    def map(self):
        w,h=44,168;data=[0]*(w*h)
        for y in range(h):data[y*w]=data[(y+1)*w-1]=100
        grid=dict(width=w,height=h,resolution=.05,origin=[0,0],yaw=0,data=data,age=.1,stale=False)
        return dict(frame='odom',layers={'static_map':grid,'map':grid,
            'robot':{'position':[1.1,.5],'yaw':1.57,'age':.1,'stale':False},
            'footprint':{'points':[[.87,.20],[1.33,.20],[1.33,.80],[.87,.80]],'age':.1,'stale':False},
            'path':{'points':[],'age':.1,'stale':False}},errors={},
            editing=dict(ready=True,base_id='mock-corridor',revision=self.revision,rectangles=self.rectangles))
    def people(self):
        people=[dict(id=17,position=[1.1,3.5],age=.1,stale=False)] if self.people_override is None else self.people_override
        value=dict(people=people,selected_id=17,
            following=False,state='idle',message='离线人物定位有效',route=[],goal=None,preview_sequence=self.preview_sequence,
            person_preview=self.preview,target_marker=self.target_marker,historical_route=[],history_age=0)
        if self.start_readiness=='auto':
            ready=bool(self.preview and self.preview.get('state')=='ready')
            value['start_readiness']={
                'single':{'ready':ready,'reason':'' if ready else '请先完成有效的人物路径规划'},
                'continuous':{'ready':ready and self.vision,'reason':'请先启用视觉障碍检测' if not self.vision else ('' if ready else '请先完成有效的人物路径规划')}}
        elif self.start_readiness is not None:value['start_readiness']=self.start_readiness
        return value
    def inflation(self):
        return dict(local={'enabled':True,'radius':0.},global_={'enabled':True,'radius':0.},consistent=True,stopped=self.stopped)

class MockServer(ThreadingHTTPServer):daemon_threads=True

def server_for(state):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def send(self,data,status=200,mime='application/json'):
            raw=json.dumps(data,ensure_ascii=False).encode() if mime=='application/json' else data
            self.send_response(status);self.send_header('Content-Type',mime);self.send_header('Content-Length',str(len(raw)))
            self.send_header('Cache-Control','no-store');self.send_header('X-Yellow-Wall-Count','12');self.send_header('X-Frame-Age','0.1');self.send_header('X-Candidate-Distance','0.9');self.end_headers()
            try:self.wfile.write(raw)
            except (BrokenPipeError,ConnectionResetError):pass
        def do_GET(self):
            path=urlsplit(self.path).path
            with state.lock:
                state.gets.append(path)
                if path in ('/','/index.html'):return self.send((ROOT/'frontend/index.html').read_bytes(),mime='text/html; charset=utf-8')
                if path.startswith('/assets/'):
                    asset=(ROOT/'frontend'/Path(path).name)
                    if asset.is_file():return self.send(asset.read_bytes(),mime='text/html; charset=utf-8' if path.endswith('.html') else ('text/css' if path.endswith('.css') else 'application/javascript'))
                    return self.send({'detail':'missing asset'},404)
                if path=='/api/status':return self.send(state.status())
                if path=='/api/config/current':return self.send(dict(capture={'fps':5,'tracking_fps':10,'duration':0,'persist_images':False},camera={'exposure_us':40000,'gain':4},follow=state.follow(),radar={}))
                if path in ('/api/config/files','/api/radar/transforms','/api/frames','/api/processed-frames'):return self.send([])
                if path in ('/api/snapshot','/api/nav2/virtual-wall','/api/nav2/vision-obstacle-preview'):return self.send(IMAGE,mime='image/svg+xml')
                if path=='/api/obstacle-map':return self.send({'detail':'模拟地图连接失败'},503) if state.fail_map else self.send(state.map())
                if path=='/api/nav2/people-map':return self.send(state.people())
                if path=='/api/nav2/inflation':
                    value=state.inflation();value['global']=value.pop('global_');return self.send(value)
                if path=='/api/nav2/vision-enabled':return self.send(dict(enabled=state.vision,stopped=state.stopped,wall_revision='mock-wall',wall_count=12,map_wall_count=12,reason='离线候选有效',diagnostics={},semantic={'enabled':True,'fresh':True,'message':'离线 YOLO','objects':[{'label':'chair','confidence':.9}]}))
                if path=='/favicon.ico':return self.send(b'',mime='image/x-icon')
                return self.send({'detail':'unmocked GET '+path},404)
        def do_POST(self):
            path=urlsplit(self.path).path;body=self.rfile.read(int(self.headers.get('Content-Length','0')))
            try:payload=json.loads(body) if body else None
            except ValueError:payload={'raw':body.decode()}
            with state.lock:
                state.posts.append({'path':path,'body':payload})
                if path=='/api/force-stop':state.stopped=True;return self.send(state.status())
                if path=='/api/force-stop/release':state.stopped=False;return self.send(state.status())
                if path=='/api/nav2/map-edit':
                    state.rectangles.append((payload or {}).get('rect'));state.revision+=1
                    return self.send(dict(ready=True,base_id='mock-corridor',revision=state.revision,rectangles=state.rectangles))
                if path=='/api/nav2/person-preview' and state.person_preview_error:
                    return self.send({'detail':state.person_preview_error},409)
                if path=='/api/nav2/person-execute' and state.person_execute_error:
                    return self.send({'detail':state.person_execute_error},409)
                if path in ('/api/nav2/person-preview','/api/nav2/point-preview','/api/nav2/preview'):
                    state.preview={'state':'ready','points':[[1.1,.5],[1.1,2.5]],'age':0.,'message':'离线整车路径已就绪'}
                    return self.send({'preview':state.preview,'target_marker':{'id':17,'position':[1.1,3.5]}} if path.endswith('person-preview') else state.preview)
                if path=='/api/nav2/vision-enabled':state.vision=payload['enabled'];return self.send({'enabled':state.vision,'stopped':state.stopped})
                return self.send({'message':'离线请求已记录，不执行实车动作','stopped':state.stopped})
    server=MockServer(('127.0.0.1',0),Handler);threading.Thread(target=server.serve_forever,daemon=True).start();return server

class CDP:
    def __init__(self,url,origin):
        self.ws=websocket.create_connection(url,timeout=10);self.counter=0;self.origin=origin;self.errors=[];self.blocked=[]
    def emit(self,method,params):
        self.counter+=1;self.ws.send(json.dumps({'id':self.counter,'method':method,'params':params}));return self.counter
    def call(self,method,params=None):
        want=self.emit(method,params or {})
        while True:
            msg=json.loads(self.ws.recv())
            if msg.get('method')=='Fetch.requestPaused':
                p=msg['params'];url=p['request']['url']
                if url.startswith(self.origin+'/'):
                    self.emit('Fetch.continueRequest',{'requestId':p['requestId']})
                else:
                    self.blocked.append(url);self.emit('Fetch.failRequest',{'requestId':p['requestId'],'errorReason':'BlockedByClient'})
            elif msg.get('method')=='Runtime.exceptionThrown':self.errors.append(msg['params']['exceptionDetails'])
            if msg.get('id')==want:
                if 'error' in msg:raise RuntimeError(msg['error'])
                return msg.get('result',{})
    def js(self,expression):
        result=self.call('Runtime.evaluate',{'expression':expression,'returnByValue':True,'awaitPromise':True})
        if 'exceptionDetails' in result:raise AssertionError(result['exceptionDetails'])
        return result.get('result',{}).get('value')
    def wait(self,predicate,timeout=12):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if self.js(predicate):return
            time.sleep(.1)
        raise AssertionError('Timed out: '+predicate)
    def click(self,selector):
        name=json.dumps(selector)
        self.js("(()=>{const e=document.querySelector("+name+"); if(!e)throw Error('missing element');for(let p=e.parentElement;p;p=p.parentElement)if(p.tagName==='DETAILS')p.open=true;e.scrollIntoView({block:'center',behavior:'instant'});})()")
        # Layout and prior optional smooth scroll must settle before real hit testing.
        for _ in range(3):self.js('document.readyState');time.sleep(.1)
        self.wait("!document.querySelector("+name+").disabled")
        point=self.js("(()=>{const e=document.querySelector("+name+");const r=e.getBoundingClientRect(),x=r.x+r.width/2,y=r.y+r.height/2,hit=document.elementFromPoint(x,y);return {x,y,disabled:e.disabled,hidden:!r.width||!r.height,hit:hit===e||e.contains(hit),hitId:hit?.id};})()")
        assert not point.get('disabled') and not point['hidden'] and point['hit'],(selector,point)
        self.call('Input.dispatchMouseEvent',dict(type='mousePressed',x=point['x'],y=point['y'],button='left',clickCount=1))
        self.call('Input.dispatchMouseEvent',dict(type='mouseReleased',x=point['x'],y=point['y'],button='left',clickCount=1))
    def click_post(self, selector, state):
        # A previous success message can still be on screen while a new request
        # is intercepted. Wait for this click's server-side observation first.
        before=len(state.posts);self.click(selector);deadline=time.monotonic()+12
        while len(state.posts)==before and time.monotonic()<deadline:
            self.js('document.readyState');time.sleep(.05)
        assert len(state.posts)>before,('No mock POST after click',selector)
        return before
    def canvas_gesture(self, points, touch=False):
        self.js("document.getElementById('obstacleMap').scrollIntoView({block:'center',behavior:'instant'})")
        # Allow the workbench's optional smooth scroll to settle before hit testing.
        for _ in range(4):self.js('document.readyState');time.sleep(.1)
        box=self.js("(()=>{const r=document.getElementById('obstacleMap').getBoundingClientRect();return {x:r.x,y:r.y,w:r.width,h:r.height};})()")
        coords=[{'x':box['x']+u*box['w'],'y':box['y']+v*box['h']} for u,v in points]
        if touch:
            self.call('Emulation.setTouchEmulationEnabled',{'enabled':True,'maxTouchPoints':1})
            self.call('Input.dispatchTouchEvent',{'type':'touchStart','touchPoints':[dict(coords[0],id=1)]})
            for xy in coords[1:]:self.call('Input.dispatchTouchEvent',{'type':'touchMove','touchPoints':[dict(xy,id=1)]})
            self.call('Input.dispatchTouchEvent',{'type':'touchEnd','touchPoints':[]})
            self.call('Emulation.setTouchEmulationEnabled',{'enabled':False})
        else:
            self.call('Input.dispatchMouseEvent',dict(type='mousePressed',button='left',buttons=1,clickCount=1,**coords[0]))
            for xy in coords[1:]:self.call('Input.dispatchMouseEvent',dict(type='mouseMoved',button='left',buttons=1,**xy))
            self.call('Input.dispatchMouseEvent',dict(type='mouseReleased',button='left',buttons=0,clickCount=1,**coords[-1]))
    def screenshot(self,path):path.write_bytes(base64.b64decode(self.call('Page.captureScreenshot',{'format':'png','captureBeyondViewport':False})['data']))

def set_map_view(cdp, *, zoom=None, pan=None, pan_x=None):
    """Drive public controls, without calling private production JS functions."""
    for control, value in [('mapZoom', zoom), ('mapPanX', pan_x), ('mapPanY', pan)]:
        if value is not None:
            cdp.js("(()=>{const e=document.getElementById("+json.dumps(control)+");e.value="+json.dumps(str(value))+";e.dispatchEvent(new Event('input',{bubbles:true}));})()")


def map_pan_offset(cdp):
    return cdp.js("({x:Number(document.getElementById('mapPanX').value),y:Number(document.getElementById('mapPanY').value)})")


def map_cell_point(cdp, x, y):
    """Independent fixture geometry: cell center on a 2.2 m × 8.4 m map.

    Return normalized canvas coordinates for real CDP input. The calculation
    uses known mock dimensions and user controls, never private render state.
    A pan bug in selection or drawing therefore changes the posted grid cell.
    """
    view=cdp.js("(()=>{const e=document.getElementById('obstacleMap');return {width:e.width,height:e.height,zoom:Number(document.getElementById('mapZoom').value),pan:Number(document.getElementById('mapPanY').value),pan_x:Number(document.getElementById('mapPanX').value)};})()")
    scale=min(view['width']/2.2,view['height']/8.4)*.9*view['zoom']
    u=.5+((x+.5)*.05-1.1+view['pan_x'])*scale/view['width']
    v=.5+(4.2-(y+.5)*.05+view['pan'])*scale/view['height']
    assert 0<u<1 and 0<v<1,('Fixture target outside viewport',x,y,view,u,v)
    return u,v


def rendered_layer_centers(cdp):
    # Real pixels catch a forgotten pan transform on robot, footprint or route.
    # Ignore antialiased boundaries and compare the solid interior colors.
    return cdp.js("""(()=>{const c=document.getElementById('obstacleMap'),d=c.getContext('2d').getImageData(0,0,c.width,c.height).data;
      const layers={robot:{n:0,x:0,y:0},route:{n:0,x:0,y:0},manual_region:{n:0,x:0,y:0}};
      for(let i=0;i<d.length;i+=4){const r=d[i],g=d[i+1],b=d[i+2];let name=null;
        if(r<20&&g>=105&&g<=130&&b>235)name='robot';
        else if(r<20&&g>=175&&g<=190&&b>=205&&b<=220)name='route';
        else if(r===17&&g===17&&b===17)name='manual_region';
        if(name){layers[name].n++;layers[name].x+=(i/4)%c.width;layers[name].y+=Math.floor(i/4/c.width);}}
      for(const layer of Object.values(layers))for(const axis of ['x','y'])layer[axis]=layer.n?layer[axis]/layer.n:null;
      return layers;
    })()""")


def check_map_pan(cdp,state,report):
    """View controls must never issue motion/edit requests or move world data."""
    cdp.wait("['mapPanX','mapPanY'].every(id=>document.getElementById(id)&&!document.getElementById(id).disabled)")
    before=len(state.posts)
    # Make a validated mock route visible, so vehicle, route and hand-drawn
    # region alignment can be checked against real pixels on both axes.
    state.preview={'state':'ready','points':[[1.1,.5],[1.1,2.5]],'age':0.,'message':'离线路线仅用于检查地图平移'}
    state.rectangles=[[20,90,24,94]]
    cdp.wait("window.__navPreviewData?.state==='ready' && window.__navMapData?.layers?.robot && window.__navMapData.editing.rectangles.length===1")
    cdp.js("window.__panWorldBefore=JSON.stringify({map:window.__navMapData,preview:window.__navPreviewData})")
    set_map_view(cdp,zoom=1,pan=0,pan_x=0)
    initial=rendered_layer_centers(cdp)
    assert all(layer['n']>20 for layer in initial.values()),initial
    set_map_view(cdp,pan=-1,pan_x=-.5)
    shifted=rendered_layer_centers(cdp)
    deltas={name:{axis:shifted[name][axis]-initial[name][axis] for axis in ['x','y']} for name in initial}
    expected={'x':-.5*600/8.4*.9,'y':-600/8.4*.9}
    assert all(abs(delta[axis]-expected[axis])<1.5 for delta in deltas.values() for axis in expected),(deltas,expected)
    set_map_view(cdp,zoom=1.4)
    assert map_pan_offset(cdp)=={'x':-.5,'y':-1},'Zoom must retain both offsets in meters'
    cdp.click('#mapPanUp')
    assert map_pan_offset(cdp)=={'x':-.5,'y':-1.25}
    cdp.click('#mapPanDown')
    assert map_pan_offset(cdp)=={'x':-.5,'y':-1}
    cdp.click('#mapPanLeft')
    assert map_pan_offset(cdp)=={'x':-.75,'y':-1}
    cdp.click('#mapPanRight')
    assert map_pan_offset(cdp)=={'x':-.5,'y':-1}
    cdp.click('#mapReset')
    assert map_pan_offset(cdp)=={'x':0,'y':0} and cdp.js("Number(document.getElementById('mapZoom').value)")==1
    gesture_results=[]
    for name,width,height,touch in [('mouse',1440,1000,False),('touch',390,844,True)]:
        cdp.call('Emulation.setDeviceMetricsOverride',dict(width=width,height=height,deviceScaleFactor=1,mobile=touch))
        set_map_view(cdp,zoom=1,pan=-1,pan_x=-.6)
        cdp.canvas_gesture([(.45,.40),(.50,.50)],touch)
        pan=map_pan_offset(cdp)
        # Independent fixture scale: .05 of canvas width moves .7 m, while
        # .10 of canvas height moves .9333 m. Both stay clear of pan limits.
        expected_drag={'x':-.6+.05*900/(600/8.4*.9),'y':-1+.1*600/(600/8.4*.9)}
        assert all(abs(pan[axis]-expected_drag[axis])<.04 for axis in expected_drag),(name,pan,expected_drag)
        cdp.canvas_gesture([(.45,.50),(.50,.50)],touch)
        horizontal=map_pan_offset(cdp)
        assert abs(horizontal['x']-(pan['x']+.05*900/(600/8.4*.9)))<.04,(name,'horizontal drag did not shift X',horizontal,pan)
        assert abs(horizontal['y']-pan['y'])<.02,(name,'horizontal drag changed Y',horizontal,pan)
        gesture_results.append(name)
    # Periodic map refreshes must preserve both user-selected view offsets.
    set_map_view(cdp,pan=-.75,pan_x=.4)
    count=cdp.js("window.__navMapCount")
    cdp.wait("window.__navMapCount>"+str(count))
    assert map_pan_offset(cdp)=={'x':.4,'y':-.75},'Map polling reset the view'
    # Only a primary left-button pointer is a view drag. These deliberately
    # synthetic guard events avoid opening the browser's native context menu.
    cdp.js("""(()=>{const c=document.getElementById('obstacleMap'),r=c.getBoundingClientRect();
      for(const p of [{button:2,buttons:2,isPrimary:true},{button:0,buttons:1,isPrimary:false}]){
        c.dispatchEvent(new PointerEvent('pointerdown',{...p,pointerId:71,clientX:r.x+r.width/2,clientY:r.y+r.height/2,bubbles:true}));
        c.dispatchEvent(new PointerEvent('pointermove',{...p,pointerId:71,clientX:r.x+r.width/2+40,clientY:r.y+r.height/2+40,bubbles:true}));
        c.dispatchEvent(new PointerEvent('pointerup',{...p,buttons:0,pointerId:71,bubbles:true}));}
    })()""")
    assert map_pan_offset(cdp)=={'x':.4,'y':-.75},'Right/non-primary pointer panned the view'
    assert cdp.js("window.__panWorldBefore===JSON.stringify({map:window.__navMapData,preview:window.__navPreviewData})"),'Pan changed world map/robot/route data'
    assert len(state.posts)==before,('Map panning must be read-only',state.posts[before:])
    set_map_view(cdp,zoom=1,pan=0,pan_x=0)
    report['map_pan']={'rendered_xy_delta_px':deltas,'zoom_preserves_both_meters':True,
        'four_buttons_and_reset':True,'diagonal_and_horizontal_drag':gesture_results,'poll_preserves_both_axes':True,
        'right_and_non_primary_ignored':True,'world_data_unchanged':True,'no_post':True}


def check_person_start_feedback(cdp,state,report,output):
    """Keep failures reviewable while periodic person snapshots keep arriving."""
    cdp.wait("!!document.getElementById('nav2PersonActionError') && !!document.getElementById('nav2PersonStartReason')")
    assert cdp.js("document.getElementById('nav2PersonActionError').getAttribute('role')==='alert'")
    selector='#nav2PersonPanel input[type=checkbox]'
    assert cdp.js("document.querySelector("+json.dumps(selector)+").checked"),'Fixture must begin in continuous mode'
    cases=[('preview','person_preview_error','#nav2PersonPreview','/api/nav2/person-preview','离线规划拒绝：人物历史点已经过期'),
           ('execute','person_execute_error','#nav2PersonStart','/api/nav2/person-execute','离线启动拒绝：视觉障碍数据尚未就绪')]
    results=[]
    for name,attribute,button,path,reason in cases:
        setattr(state,attribute,reason)
        before=cdp.click_post(button,state)
        predicate="(()=>{const e=document.getElementById('nav2PersonActionError');return !e.hidden&&e.textContent.includes("+json.dumps(reason)+")&&e.getBoundingClientRect().height>0;})()"
        cdp.wait(predicate)
        count=cdp.js('window.__navMapCount')
        cdp.wait('window.__navMapCount>='+str(count+2))
        assert cdp.js(predicate),('Polling overwrote explicit request failure',name)
        assert [post['path'] for post in state.posts[before:]]==[path],state.posts[before:]
        cdp.screenshot(output/('person_'+name+'_error.png'))
        # The next explicit successful attempt clears the error; a timer alone
        # cannot clear it. Each click issues exactly one intended mock request.
        setattr(state,attribute,None)
        cdp.click_post(button,state)
        cdp.wait("(()=>{const e=document.getElementById('nav2PersonActionError');return e.hidden||!e.textContent.trim();})()")
        assert [post['path'] for post in state.posts[before:]]==[path,path],state.posts[before:]
        results.append(name+' 409 visible after two polls; successful retry clears it')

    reason='离线条件未满足：连续追踪等待视觉检查'
    state.start_readiness={'single':{'ready':True,'reason':''},'continuous':{'ready':False,'reason':reason}}
    cdp.wait("document.getElementById('nav2PersonStart').disabled && document.getElementById('nav2PersonStartReason').textContent.includes("+json.dumps(reason)+")")
    assert cdp.js("document.getElementById('nav2PersonStartReason').getBoundingClientRect().height>0")
    before=len(state.posts)
    # Native HTMLElement.click respects disabled controls. Dispatching an
    # artificial click event would bypass the browser behavior under test.
    cdp.js("document.getElementById('nav2PersonStart').click()")
    count=cdp.js('window.__navMapCount');cdp.wait('window.__navMapCount>='+str(count+2))
    assert len(state.posts)==before,('Disabled start issued a request',state.posts[before:])
    cdp.click(selector)
    cdp.wait("!document.getElementById('nav2PersonStart').disabled")
    assert len(state.posts)==before,('Changing follow mode issued a request',state.posts[before:])
    cdp.click_post('#nav2PersonStart',state)
    cdp.wait("document.getElementById('nav2PersonPanel').textContent.includes('离线请求已记录')")
    assert state.posts[-1]['path']=='/api/nav2/person-execute' and state.posts[-1]['body']['continuous'] is False,state.posts[-1]
    cdp.click(selector)
    cdp.wait("document.getElementById('nav2PersonStart').disabled")

    # An older backend omits start_readiness. It must show an upgrade notice,
    # rather than guess that a visible target implies safe execution readiness.
    state.start_readiness=None
    missing="document.getElementById('nav2PersonStart').disabled && document.getElementById('nav2PersonStartReason').textContent.includes('启动条件尚未同步')"
    cdp.wait(missing)
    before=len(state.posts)
    cdp.click(selector)
    cdp.wait(missing)
    cdp.js("document.getElementById('nav2PersonStart').click()")
    cdp.click(selector)
    cdp.wait(missing)
    assert len(state.posts)==before,'Missing readiness must never start movement in either mode'

    # Explicit backend readiness still distinguishes vision-gated continuous
    # following from a single planned segment when visual detection is off.
    state.start_readiness='auto';state.vision=False
    cdp.wait("!document.getElementById('navVisionEnabled').checked && document.getElementById('nav2PersonStart').disabled && document.getElementById('nav2PersonStartReason').textContent.includes('视觉')")
    before=len(state.posts)
    cdp.click(selector)
    cdp.wait("!document.getElementById('nav2PersonStart').disabled")
    assert len(state.posts)==before,'Switching follow mode must not start movement'
    state.vision=True
    cdp.wait("document.getElementById('navVisionEnabled').checked")
    cdp.click(selector)
    cdp.wait("!document.getElementById('nav2PersonStart').disabled")
    report['person_start_feedback']={'request_errors':results,'readiness_reason_visible':True,
        'disabled_start_no_post':True,'mode_selects_readiness_and_payload':True,
        'missing_readiness_disables_both_modes':True,'explicit_continuous_vision_gate':True}


def check_locked_person_segment(cdp,state,report):
    """R31: execute the approved historical sequence, without reacquiring it."""
    saved={name:getattr(state,name) for name in ['people_override','preview_sequence','target_marker','preview','start_readiness']}
    state.preview_sequence=73
    state.target_marker={'id':17,'position':[1.1,3.5]}
    state.preview={'state':'ready','points':[[1.1,.5],[1.1,2.5]],'age':0.,'message':'离线历史人物路线已锁定'}
    state.start_readiness={'single':{'ready':True,'reason':''},'continuous':{'ready':True,'reason':''}}
    observation_cases=[('missing',[]),('stale',[{'id':17,'position':[1.6,6.0],'age':4.,'stale':True}])]
    allowed=[]
    for name,people in observation_cases:
        before=len(state.posts)
        state.people_override=people
        count=cdp.js('window.__navMapCount')
        cdp.wait('window.__navMapCount>='+str(count+2))
        cdp.wait("!document.getElementById('nav2PersonStart').disabled")
        assert len(state.posts)==before,('Live observation changes triggered automatic planning/execution',name,state.posts[before:])
        cdp.click_post('#nav2PersonStart',state)
        cdp.wait("document.getElementById('nav2PersonPanel').textContent.includes('离线请求已记录')")
        assert [post['path'] for post in state.posts[before:]]==['/api/nav2/person-execute'],(name,state.posts[before:])
        assert state.posts[-1]['body']['sequence']==73,(name,'Did not execute the locked sequence',state.posts[-1])
        assert state.posts[-1]['body']['continuous'] is True,state.posts[-1]
        assert state.target_marker=={'id':17,'position':[1.1,3.5]},'Observation replaced the locked marker'
        allowed.append(name+' live observation: same historical sequence, no replanning')

    denied=[]
    for reason in ['离线校验：锁定人物 ID 与当前选择不一致','离线校验：锁定路线预览已过期，请重新规划']:
        before=len(state.posts)
        state.start_readiness={'single':{'ready':False,'reason':reason},'continuous':{'ready':False,'reason':reason}}
        cdp.wait("document.getElementById('nav2PersonStart').disabled && document.getElementById('nav2PersonStartReason').textContent.includes("+json.dumps(reason)+")")
        cdp.js("document.getElementById('nav2PersonStart').click()")
        count=cdp.js('window.__navMapCount');cdp.wait('window.__navMapCount>='+str(count+2))
        assert len(state.posts)==before,('Backend rejection bypassed',reason,state.posts[before:])
        denied.append(reason)
    # Losing a fresh person is permitted; an expired approved path is not.
    before=len(state.posts)
    state.start_readiness={'single':{'ready':True,'reason':''},'continuous':{'ready':True,'reason':''}}
    state.preview={**state.preview,'age':31.}
    cdp.wait("document.getElementById('nav2PersonStart').disabled && document.getElementById('nav2PersonStartReason').textContent.includes('30 秒')")
    cdp.js("document.getElementById('nav2PersonStart').click()")
    assert len(state.posts)==before,'Expired historical path was executed'
    for name,value in saved.items():setattr(state,name,value)
    cdp.wait("!document.getElementById('nav2PersonStart').disabled")
    report['locked_person_segment']={'allowed':allowed,'locked_sequence':73,'backend_rejections_preserved':denied,
        'preview_expiry_gate_preserved':True,'no_automatic_person_preview':True}


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',default='/tmp/nav2-layout-check');args=parser.parse_args()
    output=Path(args.output);output.mkdir(parents=True,exist_ok=True)
    state=State();server=server_for(state);origin=f'http://127.0.0.1:{server.server_port}'
    sock=socket.socket();sock.bind(('127.0.0.1',0));debug_port=sock.getsockname()[1];sock.close()
    profile='/tmp/nav2-cdp-profile-'+str(os.getpid());log=(output/'chromium.log').open('wb')
    chrome=subprocess.Popen(['/snap/bin/chromium','--headless','--no-sandbox','--disable-gpu','--disable-dev-shm-usage','--disable-background-networking','--disable-component-update','--disable-sync','--no-first-run','--no-default-browser-check','--remote-allow-origins=*','--remote-debugging-address=127.0.0.1',f'--remote-debugging-port={debug_port}',f'--user-data-dir={profile}','about:blank'],stdout=log,stderr=log)
    session=requests.Session();session.trust_env=False;cdp=None;report={}
    try:
        debug=f'http://127.0.0.1:{debug_port}'
        for _ in range(150):
            try:session.get(debug+'/json/version',timeout=.3).raise_for_status();break
            except requests.RequestException:time.sleep(.1)
        else:raise RuntimeError('Chromium unavailable: '+(output/'chromium.log').read_text())
        page=session.put(debug+'/json/new?about:blank',timeout=3).json();cdp=CDP(page['webSocketDebuggerUrl'],origin)
        cdp.call('Page.enable');cdp.call('Runtime.enable');cdp.call('Network.enable')
        cdp.call('Fetch.enable',{'patterns':[{'urlPattern':'http://*'},{'urlPattern':'https://*'}]})
        template_ids=re.findall(r'\bid="([^"]+)"',(ROOT/'frontend/index.html').read_text())
        # Capture before the optional panel-layout script is requested. Dynamic
        # loader bookkeeping is intentionally excluded; every original control
        # and the pre-existing person buttons remain identity-checked.
        cdp.call('Page.addScriptToEvaluateOnNewDocument',{'source':"""
          window.addEventListener('nav2-map-snapshot',e=>{window.__navMapData=e.detail;window.__navMapCount=(window.__navMapCount||0)+1;});
          window.addEventListener('nav2-preview',e=>window.__navPreviewData=e.detail);
          document.addEventListener('DOMContentLoaded',()=>{
            const ids=TEMPLATE_IDS.concat(['nav2PersonPanel','nav2PersonPreview','nav2PersonStart','nav2PersonEnd']);
            window.__originalNodes=Object.fromEntries(ids.map(id=>[id,document.getElementById(id)]).filter(([,e])=>e));
            const main=document.querySelector('main'),panel=document.querySelector('.obstacle-map-panel');
            window.__mainNodes=Array.from(main.childNodes);
            const outside=Array.from(document.body.querySelectorAll('[id],main>section,main>header')).filter(e=>e!==panel&&!panel.contains(e));
            window.__outsideLocations=outside.map(e=>({e,parent:e.parentNode,previous:e.previousElementSibling,next:e.nextElementSibling}));
            window.__originalPanel=panel;
          },{once:true});
        """.replace('TEMPLATE_IDS',json.dumps(template_ids))})
        cdp.call('Emulation.setDeviceMetricsOverride',{'width':1440,'height':1000,'deviceScaleFactor':1,'mobile':False})
        cdp.call('Page.navigate',{'url':origin+'/'})
        cdp.wait("document.getElementById('mapStatus')?.textContent.includes('静态地图') && document.getElementById('live')?.naturalWidth>0")
        cdp.wait("Boolean(document.getElementById('nav2Panel') && document.getElementById('nav2PersonActions'))")
        # Pump CDP so intercepted periodic requests complete; loading/reparenting may never POST.
        for _ in range(12):cdp.js('document.readyState');time.sleep(.1)
        assert not state.posts,('automatic POST on load',state.posts)
        report['baseline_no_post']=True
        report['initial_ids']=cdp.js("(()=>{let counts={};document.querySelectorAll('[id]').forEach(e=>counts[e.id]=(counts[e.id]||0)+1);return Object.entries(counts).filter(([id,n])=>n!==1);})()")
        assert not report['initial_ids'],report['initial_ids']
        report['original_nodes_preserved']=cdp.js("Boolean(window.__originalNodes?.live) && Object.entries(window.__originalNodes).every(([id,e])=>document.getElementById(id)===e)")
        assert report['original_nodes_preserved'],cdp.js("Object.entries(window.__originalNodes||{}).filter(([id,e])=>document.getElementById(id)!==e).map(([id,e])=>({id,old:e.tagName,new:document.getElementById(id)?.tagName}))")
        assert cdp.js("document.getElementById('nav2Panel')===window.__originalPanel"),'Original panel must be reused'
        location_checks="""(()=>{
          const current=Array.from(document.querySelector('main').childNodes);
          return {main_order:current.length===__mainNodes.length&&current.every((e,i)=>e===__mainNodes[i]),
            outside_moved:__outsideLocations.filter(s=>s.e.parentNode!==s.parent||s.e.previousElementSibling!==s.previous||s.e.nextElementSibling!==s.next).map(s=>s.e.id||s.e.className),
            protected:['live','forceStop','releaseStop','motionReason','cameraForm','manualPad'].every(id=>!document.getElementById(id).closest('#nav2Panel'))};
        })()"""
        report['original_page_locations']=cdp.js(location_checks)
        assert report['original_page_locations']['main_order'] and report['original_page_locations']['protected']
        assert not report['original_page_locations']['outside_moved'],report['original_page_locations']
        report['css_scope']=cdp.js("""(()=>{
          const sheets=Array.from(document.styleSheets).filter(s=>s.href&&new URL(s.href).pathname==='/assets/nav2-panel.css');
          const selectors=[];let forbidden=[];
          function walk(rules){for(const r of rules){if(r.selectorText)selectors.push(r.selectorText);else if(r.cssRules)walk(r.cssRules);else forbidden.push(r.cssText);}}
          for(const sheet of sheets)walk(sheet.cssRules);
          function split(s){const parts=[];let depth=0,last=0;for(let i=0;i<s.length;i++){if('(['.includes(s[i]))depth++;else if(')]'.includes(s[i]))depth--;else if(s[i]===','&&!depth){parts.push(s.slice(last,i).trim());last=i+1;}}parts.push(s.slice(last).trim());return parts;}
          return {stylesheets:sheets.length,selectors:selectors.length,unscoped:selectors.flatMap(split).filter(s=>!/^#nav2Panel(?:\s|[.:#\[]|$)/.test(s)),forbidden};
        })()""")
        assert report['css_scope']['stylesheets']==1 and report['css_scope']['selectors']>0,report['css_scope']
        assert not report['css_scope']['unscoped'] and not report['css_scope']['forbidden'],report['css_scope']
        for label,width,height,mobile in [('desktop',1440,1000,False),('mobile',390,844,True),('desktop_again',1440,1000,False)]:
            cdp.call('Emulation.setDeviceMetricsOverride',dict(width=width,height=height,deviceScaleFactor=1,mobile=mobile))
            cdp.js("document.getElementById('nav2Panel').scrollIntoView({block:'start',behavior:'instant'})");time.sleep(.2)
            dims=cdp.js("(()=>{const p=document.getElementById('nav2Panel');return {viewport:innerWidth,page_scroll:document.documentElement.scrollWidth,panel_width:p.clientWidth,panel_scroll:p.scrollWidth,media:['wallImage','visionPreviewImage'].map(id=>({id,inside:!!document.getElementById(id)?.closest('#nav2Media')})),live_stays_outside:!document.getElementById('live').closest('#nav2Panel')};})()")
            report[label]=dims;cdp.screenshot(output/(label+'.png'))
            # Old page overflow is out of scope; this change must contain its own panel.
            assert dims['panel_scroll']<=dims['panel_width']+1,(label,dims)
            assert all(e['inside'] for e in dims['media']) and dims['live_stays_outside'],dims
        check_map_pan(cdp,state,report)
        before=cdp.click_post('#forceStop',state);cdp.wait("document.getElementById('motionMessage').textContent.includes('已强制停止')")
        assert [p['path'] for p in state.posts[before:]]==['/api/force-stop'],state.posts[before:]
        report['force_stop_exactly_once']=True
        # Reparenting must leave both old preview modules functional.
        before=len(state.posts)
        cdp.click('#wallEnabled');cdp.click('#yellowWallEnabled');cdp.click('#visionPreviewEnabled')
        cdp.wait("['live','wallImage','visionPreviewImage'].every(id=>{const e=document.getElementById(id);return !e.hidden&&e.naturalWidth>0;})")
        assert len(state.posts)==before,('preview toggles must be read-only',state.posts[before:])
        report['all_media_streams_visible']=True
        # Show every media card and both workflows at desktop/phone widths.
        for name,selector,width,height,mobile in [
                ('desktop_media','#nav2Media',1440,1000,False),
                ('desktop_actions','#nav2Actions',1440,1000,False),
                ('mobile_media','#nav2Media',390,844,True),
                ('mobile_actions','#nav2Actions',390,844,True)]:
            cdp.call('Emulation.setDeviceMetricsOverride',dict(width=width,height=height,deviceScaleFactor=1,mobile=mobile))
            cdp.js("document.querySelector("+json.dumps(selector)+").scrollIntoView({block:'start',behavior:'instant'})")
            for _ in range(3):cdp.js('document.readyState');time.sleep(.1)
            cdp.screenshot(output/(name+'.png'))

        # Real pointer events, without replacing the original canvas handlers.
        report['pointer_checks']=[]
        for label,width,height,touch in [('mouse',1440,1000,False),('touch',390,844,True)]:
            cdp.call('Emulation.setDeviceMetricsOverride',dict(width=width,height=height,deviceScaleFactor=1,mobile=touch))
            # Select an exact known world cell after both zooming and panning.
            # A selection gesture must not also start view dragging.
            set_map_view(cdp,zoom=1.4,pan=-1,pan_x=.4)
            cdp.click('#pointPick');cdp.canvas_gesture([map_cell_point(cdp,22,100)],touch)
            assert map_pan_offset(cdp)=={'x':.4,'y':-1},(label,'point selection panned map')
            cdp.wait("!document.getElementById('pointPreview').disabled")
            before=cdp.click_post('#pointPreview',state)
            cdp.wait("document.getElementById('pointMessage').textContent.includes('离线整车路径')")
            assert [p['path'] for p in state.posts[before:]]==['/api/nav2/point-preview'],state.posts[before:]
            assert state.posts[-1]['body']['base_id']=='mock-corridor'
            assert (state.posts[-1]['body']['x'],state.posts[-1]['body']['y'])==(22,100),('Pan/zoom selected wrong world cell',label,state.posts[-1])
            before=cdp.click_post('#pointStart',state)
            cdp.wait("document.getElementById('pointMessage').textContent.includes('离线请求已记录')")
            cdp.click_post('#pointStop',state);cdp.wait("document.getElementById('pointMessage').textContent.includes('已强停并取消')")
            assert [p['path'] for p in state.posts[before:]]==['/api/nav2/point-start','/api/nav2/point-stop'],state.posts[before:]
            cdp.click('#mapEditToggle')
            cdp.wait("document.getElementById('obstacleMap').classList.contains('editing')")
            cdp.js("(()=>{for(const [id,value] of [['mapDrawTool','brush'],['mapBrushWidth','1']]){const e=document.getElementById(id);e.value=value;e.dispatchEvent(new Event('change',{bubbles:true}));}})()")
            brush_cells=[(20,93),(22,98),(24,103)]
            stroke=[map_cell_point(cdp,*cell) for cell in brush_cells]
            # Continue an already-started stroke outside the map but inside the
            # canvas. The last segment must reach/clamp to edge cell x=43.
            stroke.append((.90,stroke[-1][1]));brush_cells.append((43,103))
            cdp.canvas_gesture(stroke,touch)
            assert map_pan_offset(cdp)=={'x':.4,'y':-1},(label,'brush gesture panned map')
            cdp.wait("!document.getElementById('mapEditApply').disabled")
            before=cdp.click_post('#mapEditApply',state)
            cdp.wait("document.getElementById('mapEditMessage').textContent.includes('已保存并发布') && document.getElementById('mapEditApply').disabled")
            assert [p['path'] for p in state.posts[before:]]==['/api/nav2/map-edit'],state.posts[before:]
            cells={tuple(cell) for cell in state.posts[-1]['body']['rect']['cells']}
            assert len(cells)>3 and all(cell in cells for cell in brush_cells),('Pan/zoom brush missed expected cells',label,state.posts[-1])
            # Existing applyEdit intentionally leaves drawing mode while it publishes.
            cdp.wait("!document.getElementById('mapEditToggle').disabled && !document.getElementById('obstacleMap').classList.contains('editing')")
            report['pointer_checks'].append(label+' exact cells after XY pan/zoom + preview + start/stop + brush through map edge; pick/edit exclude panning')
        # Exercise the independent person workflow, still against local mocks only.
        before=cdp.click_post('#nav2PersonPreview',state)
        cdp.wait("!document.getElementById('nav2PersonStart').disabled")
        cdp.click_post('#nav2PersonStart',state);cdp.wait("document.getElementById('nav2PersonPanel').textContent.includes('离线请求已记录')")
        assert [p['path'] for p in state.posts[before:]]==['/api/nav2/person-preview','/api/nav2/person-execute'],state.posts[before:]
        report['person_workflow_single_posts']=True
        check_person_start_feedback(cdp,state,report,output)
        check_locked_person_segment(cdp,state,report)
        # Collapsed settings are discoverable and checkbox text remains beside its box.
        cdp.js("document.querySelectorAll('#nav2Settings details').forEach(e=>e.open=true)")
        report['checkbox_alignment']=cdp.js("""(()=>{const out=[];document.querySelectorAll('#nav2Panel input[type=checkbox]').forEach(c=>{const r=c.getBoundingClientRect(),label=c.closest('label');if(!r.width||!r.height)return;const text=Array.from(label.childNodes).find(n=>n.nodeType===3&&n.textContent.trim());if(!text)return;const range=document.createRange();range.selectNodeContents(text);const t=Array.from(range.getClientRects()).find(v=>v.width>0);out.push({id:c.id||'continuous',ok:!!t&&t.left>=r.right&&Math.abs(t.top-r.top)<12,box:[r.left,r.top,r.width,r.height],text:t?[t.left,t.top,t.width,t.height]:null});});return out;})()""")
        assert all(x['ok'] for x in report['checkbox_alignment']),report['checkbox_alignment']
        cdp.js("document.querySelectorAll('#nav2Settings details').forEach(e=>e.open=false)")
        # Release disables map editing and selecting; force-stop re-enables them.
        before=cdp.click_post('#releaseStop',state)
        cdp.wait("document.getElementById('pointPick').disabled && document.getElementById('mapEditToggle').disabled")
        assert [p['path'] for p in state.posts[before:]]==['/api/force-stop/release']
        before=cdp.click_post('#forceStop',state)
        cdp.wait("!document.getElementById('pointPick').disabled && !document.getElementById('mapEditToggle').disabled")
        assert [p['path'] for p in state.posts[before:]]==['/api/force-stop']
        report['stop_gates_preserved']=True
        state.preview={'state':'error','points':[],'age':0.,'message':'模拟终点不可通行'}
        cdp.wait("!document.getElementById('mapPathValidity').hidden && document.getElementById('mapPathValidity').textContent.includes('模拟终点不可通行')")
        report['invalid_route_visible']=cdp.js("document.getElementById('mapPathValidity').getBoundingClientRect().height>0")
        assert report['invalid_route_visible']
        state.fail_map=True
        cdp.wait("document.getElementById('mapStatus').textContent.includes('地图连接失败') && document.getElementById('pointPick').disabled")
        report['map_error_visible']=cdp.js("document.getElementById('mapStatus').getBoundingClientRect().height>0")
        assert report['map_error_visible']
        state.fail_map=False;cdp.wait("!document.getElementById('pointPick').disabled")
        report['final_node_identity']=cdp.js("Object.entries(window.__originalNodes).every(([id,e])=>document.getElementById(id)===e)")
        assert report['final_node_identity']
        report['final_page_locations']=cdp.js(location_checks)
        assert report['final_page_locations']['main_order'] and report['final_page_locations']['protected']
        assert not report['final_page_locations']['outside_moved'],report['final_page_locations']
        report['runtime_errors']=cdp.errors;report['external_requests_blocked']=cdp.blocked
        assert not cdp.errors,cdp.errors
        (output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(report,ensure_ascii=False,indent=2))
    except Exception:
        if cdp:
            print('DEBUG',cdp.js("({state:document.readyState,url:location.href,body:document.body?.innerText.slice(0,2000),live:document.getElementById('live')?.naturalWidth})"))
            print('ERRORS',cdp.errors[:2],'BLOCKED',cdp.blocked,'GETS',state.gets[:30])
            cdp.screenshot(output/'failure.png')
        raise
    finally:
        if cdp:
            try:cdp.call('Browser.close')
            except Exception:pass
            cdp.ws.close()
        chrome.terminate()
        try:chrome.wait(timeout=5)
        except subprocess.TimeoutExpired:chrome.kill();chrome.wait(timeout=3)
        server.shutdown();server.server_close();log.close();session.close()
        # Snap has a private /tmp; remove only this run's profile in that same namespace.
        cleanup=subprocess.run(['snap','run','--shell','chromium','-c','rm -rf -- "$1"','cleanup',profile],capture_output=True,text=True,timeout=10)
        shutil.rmtree(profile,ignore_errors=True)
        if cleanup.returncode:raise RuntimeError('Chromium profile cleanup failed: '+cleanup.stderr)


if __name__=='__main__':main()
