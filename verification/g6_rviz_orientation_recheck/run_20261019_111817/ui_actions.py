"""RViz diagnostic UI actions; never save over original RViz configuration."""
from Xlib import X, XK, display
from Xlib.ext import xtest
import time
D=display.Display(); root=D.screen().root
TARGET_TITLE = None

def window():
    def walk(w):
        try:
            name=w.get_wm_name() or ''
            if name.endswith(' - RViz') and (TARGET_TITLE is None or TARGET_TITLE in name):return w
            for child in w.query_tree().children:
                match=walk(child)
                if match:return match
        except Exception:pass
    w=walk(root)
    if not w:raise RuntimeError('RViz window not found')
    return w

def focus():
    w=window();w.configure(stack_mode=X.Above);w.set_input_focus(X.RevertToParent,X.CurrentTime);D.sync();return w

def key(name,down=True):
    code=D.keysym_to_keycode(XK.string_to_keysym(name));xtest.fake_input(D,X.KeyPress if down else X.KeyRelease,code);D.sync()

def combo(*names):
    for name in names:key(name)
    for name in reversed(names):key(name,False)
    time.sleep(.3)

def text(value):
    for c in value:
        pairs=D.keysym_to_keycodes(ord(c));pairs=[p for p in pairs if p[1]<2]
        if not pairs:raise RuntimeError('No key for '+repr(c))
        code,index=pairs[0]
        if index==1:key('Shift_L')
        xtest.fake_input(D,X.KeyPress,code);xtest.fake_input(D,X.KeyRelease,code)
        if index==1:key('Shift_L',False)
    D.sync();time.sleep(.3)

def click(x,y,count=1):
    w=window();p=root.translate_coords(w,0,0)
    for _ in range(count):
        xtest.fake_input(D,X.MotionNotify,x=p.x+x,y=p.y+y)
        xtest.fake_input(D,X.ButtonPress,1);xtest.fake_input(D,X.ButtonRelease,1)
    D.sync();time.sleep(.3)

if __name__=='__main__':
    w=focus();w.configure(x=70,y=70);D.sync();time.sleep(.5)
    p=root.translate_coords(w,0,0);print('RViz window',hex(w.id),'origin',p.x,p.y,'geometry',w.get_geometry()._data)
    combo('Control_L','Shift_L','s')
