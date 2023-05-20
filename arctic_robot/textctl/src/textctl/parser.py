import ply.lex as lex
import ply.yacc as yacc


tokens = ('STOP', 'CONTINUE', 'MOVE', 'TO', 'VELOCITY', 'ROTATE',
          'POS', 'GET', 'CAMERA', 'DEPTH', 'AIM_CAMERA',
          'OBJECTS', 'PATROL', 'FNUM', 'INUM', 'UINUM')

t_STOP = 'stop'
t_CONTINUE = 'continue'
t_MOVE = 'move'
t_TO = 'to'
t_VELOCITY = 'velocity'
t_ROTATE = 'rotate'
t_GET = 'get'
t_POS = 'pos'
t_CAMERA = 'camera'
t_DEPTH = 'depth'
t_AIM_CAMERA = 'aim_camera'
t_OBJECTS = 'objects'
t_PATROL = 'patrol'
t_FNUM = '[+-]?[0-9]+\.[0-9]*'
t_INUM = '[+-][0-9]+'
t_UINUM = '[0-9]+'

t_ignore = " \t"

def t_newline(t):
    r'\n+'
    t.lexer.lineno += t.value.count("\n")
    
def t_error(t):
    print("Illegal character '%s'" % t.value[0])
    t.lexer.skip(1)

lexer = lex.lex()

precedence = ()

names = { }

def p_cmd_1(p):
    'cmd : STOP'
    p[0] = ('STOP',)

def p_cmd_2(p):
    'cmd : CONTINUE'
    p[0] = ('CONTINUE',)

def p_cmd_3(p):
    'cmd : VELOCITY fval fval'
    p[0] = ('VELOCITY', p[2], p[3])

def p_cmd_4(p):
    'cmd : MOVE fval'
    p[0] = ('MOVE', p[2])

def p_cmd_5(p):
    'cmd : ROTATE fval'
    p[0] = ('ROTATE', p[2])

def p_cmd_6(p):
    'cmd : TO fval fval fval'
    p[0] = ('TO', p[2], p[3], p[4])

def p_cmd_7(p):
    'cmd : AIM_CAMERA fval fval'
    p[0] = ('AIM_CAMERA', p[2], p[3])

def p_cmd_8(p):
    'cmd : GET POS'
    p[0] = ('GET', 'POS')

def p_cmd_9(p):
    'cmd : GET CAMERA UINUM'
    p[0] = ('GET', 'CAMERA', p[3])

def p_cmd_10(p):
    'cmd : GET DEPTH UINUM'
    p[0] = ('GET', 'DEPTH', p[3])

def p_cmd_11(p):
    'cmd : GET OBJECTS'
    p[0] = ('GET', 'OBJECTS')

def p_cmd_12(p):
    'cmd : PATROL'
    p[0] = ('PATROL',)

def p_fval_1(p):
    'fval : FNUM'
    p[0] = p[1]

def p_fval_2(p):
    'fval : INUM'
    p[0] = p[1]

def p_fval_3(p):
    'fval : UINUM'
    p[0] = p[1]


def p_error(p):
    global command_parser
    if not p:
        print("Unexpected end of File!")
        return
    while True:
        tok = command_parser.token()
        print('ERROR: ' + str(tok))
        if not tok: break
    command_parser.restart()

command_parser = yacc.yacc()
   
    
