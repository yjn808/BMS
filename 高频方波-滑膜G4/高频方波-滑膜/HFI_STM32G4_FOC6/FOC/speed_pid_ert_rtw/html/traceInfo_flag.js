function TraceInfoFlag() {
    this.traceFlag = new Array();
    this.traceFlag["speed_pid.c:72c26"]=1;
    this.traceFlag["speed_pid.c:72c42"]=1;
    this.traceFlag["speed_pid.c:78c36"]=1;
    this.traceFlag["speed_pid.c:78c47"]=1;
    this.traceFlag["speed_pid.c:81c28"]=1;
    this.traceFlag["speed_pid.c:83c35"]=1;
    this.traceFlag["speed_pid.c:97c11"]=1;
    this.traceFlag["speed_pid.c:97c31"]=1;
    this.traceFlag["speed_pid.c:97c55"]=1;
    this.traceFlag["speed_pid.c:98c15"]=1;
    this.traceFlag["speed_pid.c:98c41"]=1;
    this.traceFlag["speed_pid.c:98c53"]=1;
}
top.TraceInfoFlag.instance = new TraceInfoFlag();
function TraceInfoLineFlag() {
    this.lineTraceFlag = new Array();
    this.lineTraceFlag["speed_pid.c:72"]=1;
    this.lineTraceFlag["speed_pid.c:78"]=1;
    this.lineTraceFlag["speed_pid.c:81"]=1;
    this.lineTraceFlag["speed_pid.c:82"]=1;
    this.lineTraceFlag["speed_pid.c:83"]=1;
    this.lineTraceFlag["speed_pid.c:84"]=1;
    this.lineTraceFlag["speed_pid.c:86"]=1;
    this.lineTraceFlag["speed_pid.c:97"]=1;
    this.lineTraceFlag["speed_pid.c:98"]=1;
    this.lineTraceFlag["speed_pid.c:103"]=1;
}
top.TraceInfoLineFlag.instance = new TraceInfoLineFlag();
