"""Per-thread persistent HTTP connections; never retry a movement request."""
import collections,http.client,json,select,socket,threading,time
from urllib.parse import urlsplit

class ControlTransportError(RuntimeError):pass

class ControlHTTP:
    def __init__(self,url,token):
        parts=urlsplit(url)
        if parts.scheme!='http' or parts.hostname not in ('127.0.0.1','localhost'):
            raise ValueError('控制接口必须是本机转发地址')
        self.host=parts.hostname;self.port=parts.port;self.token=token
        self.local=threading.local();self.events=collections.deque(maxlen=120);self.event_lock=threading.Lock()
    def request(self,path,body=None,timeout=1.):
        begun=time.monotonic();method='GET' if body is None else 'POST';status=None;error=None
        connection=getattr(self.local,'connection',None)
        if connection is not None and connection.sock is not None:
            # Inspect before sending: a server may have closed an idle keepalive
            # socket. Reopening here cannot replay a POST, since none was sent.
            try:
                stale=begun-getattr(self.local,'last_used',begun)>2.
                readable=bool(select.select([connection.sock],[],[],0)[0])
            except (OSError,ValueError):stale=True;readable=True
            if stale or readable:
                connection.close();self.local.connection=None;connection=None
        if connection is None:
            connection=http.client.HTTPConnection(self.host,self.port,timeout=timeout)
            self.local.connection=connection
        connection.timeout=timeout
        if connection.sock is not None:connection.sock.settimeout(timeout)
        try:
            connection.request(method,path,body=None if body is None else json.dumps(body).encode(),
                headers={'Content-Type':'application/json','X-Teleop-Token':self.token})
            response=connection.getresponse();status=response.status;payload=response.read()
            if status>=400:
                detail=payload.decode(errors='replace').replace(self.token,'[redacted]')[:600]
                raise RuntimeError(f'机器人接口 {status}: {detail}')
            return json.loads(payload)
        except (OSError,http.client.HTTPException,ValueError) as exc:
            error=type(exc).__name__;connection.close();self.local.connection=None
            label='速度/停止接口' if path=='/teleop/cmd_vel' else '状态接口' if path=='/health' else path
            message=f'{label}响应超时（{timeout*1000:.0f} ms）' if isinstance(exc,(TimeoutError,socket.timeout)) else f'{label}连接异常：{error}'
            raise ControlTransportError(message+'；请求未自动重发') from None
        finally:
            self.local.last_used=time.monotonic()
            event=dict(at_unix_s=time.time(),method=method,path=path,elapsed_s=time.monotonic()-begun,
                       timeout_s=timeout,http_status=status,error=error)
            with self.event_lock:self.events.append(event)
    def diagnostics(self):
        with self.event_lock:return list(self.events)
