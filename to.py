import time                                                                                                     
import socket                                                                                                   
import oscserver

def start():
    t = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)                                                            
    addr = socket.getaddrinfo('192.168.1.147','2222')                                                               
    addr = addr[0][-1]                                                                                              
    t.setblocking(0) 
    t.bind(addr)     
    avgpass = 0
    try:                                                                                                     
        while True:                                                                                                     
            try:                                                                                                        
                stime = time.ticks_us()
                data, caddr = t.recvfrom(100)                                                                           
                #data = oscserver.parse_message(data)
                etime = time.ticks_us()
                print("avg pass", avgpass, "recv: ", etime - stime, " data: ", data)                                                                           
            except OSError:                                                                                                     
                pass                                                                                                    
                etime = time.ticks_us()
                avgpass = (avgpass + etime - stime)/2                                                                   
            time.sleep_ms(500)                                                                                          
    except KeyboardInterrupt:
        print("Ctrl-C - cleanup")
        t.close() 

