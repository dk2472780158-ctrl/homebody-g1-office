import contextlib,http.server,json,threading,time,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import g1_real_control as control
from g1_control_http import ControlHTTP,ControlTransportError

class TransportChecks(unittest.TestCase):
    @contextlib.contextmanager
    def server(self,delay=0.,status=200,close_after_reply=False):
        received=[]
        class Handler(http.server.BaseHTTPRequestHandler):
            protocol_version='HTTP/1.1'
            def log_message(self,*args):pass
            def handle(self):
                try:super().handle()
                except OSError:pass
            def do_GET(self):self.respond(None)
            def do_POST(self):self.respond(json.loads(self.rfile.read(int(self.headers['Content-Length']))))
            def respond(self,body):
                received.append((self.path,body,self.client_address[1]))
                time.sleep(delay)
                payload=json.dumps({'ok':True,'cmd':body} if status==200 else {'detail':'private-test-token'}).encode()
                try:
                    self.send_response(status);self.send_header('Content-Length',str(len(payload)))
                    self.end_headers();self.wfile.write(payload)
                    if close_after_reply:self.close_connection=True
                except (OSError,ValueError):pass
        server=http.server.ThreadingHTTPServer(('127.0.0.1',0),Handler)
        worker=threading.Thread(target=server.serve_forever,daemon=True);worker.start()
        transport=ControlHTTP(f'http://127.0.0.1:{server.server_port}','private-test-token')
        try:yield transport,received
        finally:
            connection=getattr(transport.local,'connection',None)
            if connection is not None:connection.close()
            server.shutdown();server.server_close();worker.join(timeout=1.)
    def test_successive_requests_reuse_the_connection_and_record_latency(self):
        with self.server() as (transport,received):
            transport.request('/health',timeout=.5)
            transport.request('/teleop/cmd_vel',{'vx':.35,'vy':0.,'wz':0.},timeout=.35)
            self.assertEqual(len(received),2)
            self.assertEqual(received[0][2],received[1][2])
            self.assertEqual(transport.diagnostics()[-1]['path'],'/teleop/cmd_vel')
            self.assertEqual(transport.diagnostics()[-1]['http_status'],200)
    def test_ambiguous_nonzero_timeout_is_never_replayed(self):
        with self.server(delay=.08) as (transport,received):
            with self.assertRaisesRegex(ControlTransportError,'响应超时.*未自动重发'):
                transport.request('/teleop/cmd_vel',{'vx':.35,'vy':0.,'wz':0.},timeout=.02)
            time.sleep(.1)
            self.assertEqual(len(received),1)
            self.assertIsNone(transport.local.connection)
            self.assertEqual(transport.diagnostics()[-1]['error'],'TimeoutError')
    def test_errors_and_diagnostics_do_not_expose_the_credential(self):
        with self.server(status=403) as (transport,received):
            with self.assertRaises(RuntimeError) as error:transport.request('/health')
            self.assertNotIn('private-test-token',str(error.exception))
            self.assertNotIn('private-test-token',json.dumps(transport.diagnostics()))
    def test_server_closed_idle_socket_is_replaced_before_sending_a_post(self):
        with self.server(close_after_reply=True) as (transport,received):
            transport.request('/health');time.sleep(.05)
            transport.request('/teleop/cmd_vel',{'vx':.35,'vy':0.,'wz':0.})
            self.assertEqual(len(received),2)
            self.assertNotEqual(received[0][2],received[1][2])
            self.assertEqual(received[1][1]['vx'],.35)
    def test_only_zero_speed_may_retry(self):
        client=object.__new__(control.Teleop);client.lock=threading.Lock();calls=[]
        def request(path,body=None,timeout=1.):
            calls.append(body)
            if len(calls)==1:raise ControlTransportError('timeout')
            return {'ok':True}
        client.request=request;client.zero()
        self.assertEqual(calls,[{'vx':0.,'vy':0.,'wz':0.}]*2)
    def test_original_error_is_not_hidden_by_a_stop_timeout(self):
        e=object.__new__(control.Executor)
        def failed_snapshot():raise RuntimeError('original localization error')
        def failed_zero():raise ControlTransportError('stop timed out')
        e.w=SimpleNamespace(snapshot=failed_snapshot)
        e.client=SimpleNamespace(health=lambda:{'armed':True,'ros':True,'vel_owner':{'owner':'teleop'}},zero=failed_zero)
        with self.assertRaisesRegex(RuntimeError,'original localization error'):e.run([{'skill':'navigate'}],'same')
        self.assertEqual(e.stop_error,'stop timed out')

if __name__=='__main__':unittest.main()
