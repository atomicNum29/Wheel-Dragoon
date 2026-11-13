const fs=require('fs');
const path=require('path');
const scan=JSON.parse(fs.readFileSync('.ua/intermediate/scan-result.json','utf8'));
const batches=JSON.parse(fs.readFileSync('.ua/intermediate/batches.json','utf8')).batches;
const types=Object.fromEntries(scan.files.map(f=>[f.path,f.fileCategory==='docs'?'document':f.fileCategory==='config'?'config':'file']));
const fileInfo={
'README.md':['Teensy 3.2 기반 RC 전륜 조향·4륜 구동 프로젝트의 진입 문서다. 네 빌드 프로필, MANUAL/PIVOT 동작, 문서 탐색 순서와 소스 설명의 검증 한계를 정리한다.',['프로젝트-개요','문서','진입점']],
'platformio.ini':['Teensy 3.2의 Arduino 빌드 환경과 CONTROL·DEBUG·TIMING 조합의 네 프로필을 정의한다. 전처리 기능 플래그와 소스 필터로 선택 모듈의 포함 여부를 결정한다.',['빌드-설정','프로필','조건부-컴파일']],
'pyproject.toml':['Python 호스트 도구의 프로젝트 메타데이터, Python 3.14 이상 요구와 pyserial 의존성을 선언한다.',['의존성','환경-설정','호스트-도구']],
'docs/README.md':['2026-11-13 소스 기준 문서의 탐색 순서와 각 기능의 코드 근거를 연결한다. 현재 소스의 빌드·테스트·실물 통신 및 업로드 바이너리 일치 여부가 미확인임을 명시한다.',['문서-목차','근거-수준','구조-안내']],
'docs/can-cli-windows.md':['수신 전용 모니터, 대화형 CAN CLI, 단발 명령 도구의 역할과 사용 조건을 설명한다. 브릿지·벤치마크 실행 조건 및 일반 주행과의 비교 한계를 구분한다.',['호스트-도구','사용-안내','진단']],
'docs/can-rx-fifo.md':['드라이버와 휠 매핑, CAN 송수신 스케줄, 응답 창과 전압 판정을 설명한다. FIFO·ISR·RAM 큐 경로 및 진단 수치의 해석 한계를 정리한다.',['can','응답-유효성','수신-큐']],
'docs/firmware-architecture.ko.md':['네 빌드 프로필과 모듈별 상태 소유권, setup/loop 실행 순서, 기능별 프로토콜 포함 여부를 설명한다. RC 제어에서 CAN 구동까지의 처리 경로를 안내한다.',['펌웨어-구조','상태-소유권','실행-흐름']],
'docs/raspberry-pi-handoff.ko.md':['Raspberry Pi 인계 시 저장소 상태와 환경을 식별하고 필요한 문서·프로필을 전달하는 절차를 설명한다.',['작업-인계','환경-설정','문서']],
'docs/rc-loss-of-control-review-2026-11-09.ko.md':['2026-11-09 조종 기능 상실 관측과 펌웨어 정적 검토의 원인 후보를 기록한다. 소스상 경로, 조건부 추론, 미확인 장비 상태를 구분하며 CAN abort·RC 입력 만료·전원 등의 후보를 실제 사고 원인으로 확정하지 않는다.',['사고-기록','정적-검토','미확인-원인']],
'docs/rc-steering.ko.md':['RC 펄스와 모드 구간, 전원·구동 권한, 차량 기하·서보 보정 및 모션 제한을 설명한다. MANUAL/PIVOT 운동학과 STOP 요청의 물리적 정지 확인 한계를 정리한다.',['rc-제어','조향','운동학']],
'docs/timing-diagnostics.md':['조건부 타이밍 진단의 누적 시간, CAN 응답 누락 분류 및 0x84 패킷을 설명한다. 출력 부하와 스냅샷 지속 등 측정값 해석 조건을 명시한다.',['타이밍','계측','can-진단']],
'docs/usb-protocol.ko.md':['AA 55 프레임의 길이·XOR 규칙과 제어·보호 리셋·상태·브릿지·벤치마크 패킷의 바이트 계약을 정의한다. 프로필별 지원 범위와 제어 권한 제약도 함께 설명한다.',['usb','프로토콜','직렬화']],
'.python-version':['호스트 도구 실행 환경에서 사용할 Python 버전을 3.14로 지정한다.',['python','버전','환경-설정']],
'.ua/config.json':['Understand-Anything 분석 결과의 출력 언어를 한국어로 지정한다. 펌웨어의 실행 설정과는 별개의 분석 도구 설정이다.',['분석-설정','한국어','지식-그래프']],
'include/README':['헤더별 공개 인터페이스·설정·운동학·CAN 역할과 모듈 내부 상태 소유 원칙을 안내한다.',['헤더-안내','인터페이스','상태-소유권']],
'lib/README':['별도 라이브러리 소스가 없는 디렉터리임을 설명하고 자체 CAN 구현 및 의존성 기준 파일로 안내한다.',['라이브러리-안내','의존성','구조-안내']],
'src/can_cli.py':['Windows와 POSIX에서 CAN 브릿지 명령, 반복 요청 벤치마크, 수신 패킷·타이밍 진단을 처리하는 대화형 호스트 CLI다. 요청 인코딩, 증분 프레임 파싱, 세션별 RTT 통계와 터미널 입력을 한 파일에서 관리한다.',['호스트-cli','can-브릿지','벤치마크','패킷-파서']],
'src/control.py':['속도·각속도 제어 패킷과 보호 리셋 패킷을 생성해 USB 시리얼로 한 번 전송한다. 제어 응답은 텍스트 한 줄로 읽으므로 바이너리 상태 패킷을 해석하는 도구는 아니다.',['단발-명령','usb','패킷-인코딩']],
'src/packet_monitor.py':['명령을 송신하지 않고 MCU 직렬 스트림에서 유효 프레임을 추출해 HEX로 표시한다. 불완전 프레임을 유지하고 길이·XOR 검사 실패 후 헤더를 다시 찾는다.',['수신-전용','패킷-검증','직렬-관측']]
};
const fn={
wait_for_activity:'Windows에서는 시리얼 버퍼와 키 입력을 폴링하고 POSIX에서는 select로 수신·입력 준비 상태를 기다린다.',
xor_checksum:'프레임 바이트를 XOR하여 1바이트 체크섬을 계산한다.',parse_int:'십진수와 접두사가 있는 숫자 문자열을 정수로 변환한다.',parse_byte:'명령 인자를 정수로 읽고 0부터 255까지의 바이트 범위를 검증한다.',parse_can_id:'명령 인자를 정수로 읽고 표준 CAN 식별자 범위를 검증한다.',
find_teensy_port:'직렬 포트 목록의 장치 이름과 설명으로 Teensy 연결 후보를 찾는다.',
build_bridge_request:'CAN 식별자·DLC·데이터·대기시간을 검증하고 0x20 브릿지 요청에 XOR 체크섬을 붙인다.',
build_benchmark_request:'요청 프레임과 응답 PID·반복 횟수·주기·시간제한을 검증하여 0x21 벤치마크 요청으로 직렬화한다.',
build_command_parser:'대화형 pid·frame·benchmark 명령의 argparse 하위 파서와 입력 옵션을 만든다.',
command_from_namespace:'파싱한 입력을 검증하고 브릿지 또는 벤치마크 명령 객체로 변환한다. 요청 PID에 따라 기본 응답 PID를 정한다.',
parse_interactive_command:'셸 형식의 입력 줄을 토큰화하고 도움말·종료·CAN 명령을 구분한다.',interactive_help:'대화형 CAN 명령과 주요 옵션의 도움말을 반환한다.',format_data:'DLC에 맞춰 데이터를 잘라내거나 0으로 채우고 16진수 문자열로 표시한다.',
format_timing_packet:'버전 1의 0x84 기록을 누적·단계별·휠별·드라이버별 진단 텍스트로 해석한다. CPU 주파수로 사이클 수를 시간 단위로 바꾼다.',
format_packet:'브릿지 응답·벤치마크 결과·타이밍 기록을 구분해 상태와 수신 데이터를 표시하며 그 외 패킷은 원시 바이트로 표시한다.',
send_command:'명령 객체에 맞는 요청 프레임을 전송하고 로그를 남긴 뒤 8비트 시퀀스를 증가시킨다.',
monitor:'시리얼 수신과 키보드 입력을 반복 처리하며 패킷 표시·벤치마크 통계·대화형 송신을 조율한다.',
add_connection_options:'포트·보드레이트·시퀀스·출력 필터·대기시간에 대한 CLI 옵션을 추가한다.',
build_main_parser:'연결 옵션과 초기 pid·frame·benchmark 명령을 갖춘 최상위 CLI 파서를 구성한다.',
main:'CLI 인자를 검증하고 포트를 선택한 뒤 모니터 루프를 실행하며 종료·오류 상태를 처리한다.',
build_command_packet:'속도와 각속도의 범위를 확인하고 milli 단위 int16, 제어 플래그, 시퀀스를 리틀엔디언 0x01 패킷으로 인코딩한다.',
build_reset_packet:'보호 매직 바이트와 시퀀스를 포함한 0x02 MCU 리셋 요청 프레임을 만든다.',
send_packet:'지정 또는 탐지된 직렬 포트를 열어 패킷을 보내고 선택적으로 응답 한 줄을 읽는다.',
send_reset:'보호 리셋 패킷을 한 번 전송하고 USB 재연결 특성을 고려해 응답을 기다리지 않는다.',
extract_packets:'버퍼에서 AA 55 헤더와 길이·XOR가 유효한 완전 프레임만 추출한다. 분할 헤더와 미완성 프레임을 보존하고 오류 시 한 바이트씩 재동기화한다.'
};
const cls={BridgeCommand:'CAN 브릿지 요청의 식별자·데이터·DLC·대기시간을 보관하는 데이터 클래스다.',BenchmarkCommand:'반복 요청 프레임, 응답 식별자·PID, 반복 횟수·주기·시간제한을 보관하는 데이터 클래스다.',BenchmarkStatistics:'하나의 벤치마크 세션에서 중복 반복 결과를 제외하고 응답별·회차별 RTT와 송수신 간격 분포, 부분 응답·실패 수를 집계한다.',CommandError:'대화형 입력 오류를 프로그램 종료 없이 보고하기 위한 예외 유형이다.',ExitRequested:'사용자 종료 명령을 모니터 루프 밖으로 전달하는 예외 유형이다.',RaisingArgumentParser:'argparse 오류를 CommandError 예외로 변환해 대화형 세션을 유지한다.',PacketStreamParser:'직렬 수신 조각을 누적해 체크섬이 유효한 패킷으로 분리하고 손상 구간을 재동기화한다.',TerminalUi:'Windows 콘솔과 POSIX 터미널의 입력·프롬프트·로그 표시를 통합하고 컨텍스트 종료 시 터미널 속성을 복원한다.'};
const methodData=[
['BenchmarkStatistics','start',99,117,'시퀀스·예상 횟수·응답 PID를 설정하고 이전 세션의 통계와 시각을 초기화한다.'],
['BenchmarkStatistics','cancel',119,136,'활성 세션 식별자와 누적 표본·시각·오류 수를 초기화한다.'],
['BenchmarkStatistics','_format_distribution',151,163,'표본 수, 최소·평균·백분위·최대·표준편차를 진단 문자열로 만든다.'],
['BenchmarkStatistics','consume',165,271,'일치하는 세션의 결과만 누적하고 중복 반복을 제외한다. 최종 결과에서 응답별·회차별 시간 분포와 성공·부분 응답·실패 요약을 생성한다.'],
['PacketStreamParser','feed',293,326,'수신 조각을 누적하여 유효 길이와 XOR를 만족하는 프레임을 반환하고 불완전 데이터는 보존한다.'],
['TerminalUi','__enter__',340,353,'대화형 터미널을 확인하고 POSIX에서는 비정규 입력 모드로 바꾼 뒤 프롬프트를 표시한다.'],
['TerminalUi','read_characters',381,397,'운영체제별 입력 API로 문자를 읽는다. Windows에서 한 번에 최대 64문자만 처리하여 직렬 수신을 과도하게 지연시키지 않도록 한다.'],
['TerminalUi','read_lines',399,428,'입력 문자에서 특수키·지우기·제어문자를 처리하고 완성된 명령 줄을 반환한다.']
];
const documents={
'README.md':['src/main.cpp','platformio.ini','include/steering_config.hpp','include/can_tx_schedule.hpp'],
'docs/README.md':['src/main.cpp','src/rc_io.cpp','src/control.cpp','src/can_control.cpp','src/flexcan0.cpp','src/serial_protocol.cpp','src/can_debug.cpp','src/can_cli.py','src/packet_monitor.py','src/control.py','src/timing_diagnostics.cpp'],
'docs/can-cli-windows.md':['src/can_cli.py','src/control.py','src/packet_monitor.py'],
'docs/can-rx-fifo.md':['src/can_control.cpp','src/flexcan0.cpp','src/md200t_can.cpp','include/can_tx_schedule.hpp'],
'docs/firmware-architecture.ko.md':['platformio.ini','src/main.cpp','src/control.cpp','src/serial_protocol.cpp','include/build_features.hpp'],
'docs/raspberry-pi-handoff.ko.md':['pyproject.toml','platformio.ini'],
'docs/rc-loss-of-control-review-2026-11-09.ko.md':['src/rc_io.cpp','src/control.cpp','src/flexcan0.cpp','src/can_control.cpp','src/serial_protocol.cpp'],
'docs/rc-steering.ko.md':['src/rc_io.cpp','src/control.cpp','include/steering_config.hpp','include/vehicle_kinematics.hpp','include/vehicle_motion.hpp'],
'docs/timing-diagnostics.md':['src/timing_diagnostics.cpp','src/can_cli.py','include/timing_diagnostics.hpp'],
'docs/usb-protocol.ko.md':['src/serial_protocol.cpp','include/serial_protocol.hpp','src/can_debug.cpp','src/control.py']
};
const weights={contains:1,exports:.8,calls:.8,documents:.5,configures:.6,related:.5};
for(const index of [5,6,7]){
 const batch=batches.find(b=>b.batchIndex===index);
 const x=JSON.parse(fs.readFileSync(`.ua/tmp/ua-file-extract-results-${index}.json`,'utf8'));
 if(x.filesUnreadable.length)throw Error('Unreadable files');
 let nodes=[],edges=[];
 const edge=(s,t,type)=>edges.push({source:s,target:t,type,direction:'forward',weight:weights[type]});
 for(const f of batch.files){
  const r=x.results.find(r=>r.path===f.path);const data=fileInfo[f.path];if(!data)throw Error(f.path);
  const type=types[f.path],id=type+':'+f.path;
  nodes.push({id,type,name:path.basename(f.path),filePath:f.path,summary:data[0],tags:data[1],complexity:(r?.nonEmptyLines??f.sizeLines)>200?'complex':(r?.nonEmptyLines??f.sizeLines)>=50?'moderate':'simple'});
  for(const item of [...(r?.functions??[]).map(a=>({...a,type:'function'})),...(r?.classes??[]).map(a=>({...a,type:'class'}))]){
   const exported=r.exports?.some(e=>e.name===item.name);if(!exported && (item.type==='function'?item.endLine-item.startLine+1<10:((item.methods?.length??0)<2&&item.endLine-item.startLine+1<20)))continue;
   let summary=item.type==='class'?cls[item.name]:fn[item.name];
   if(f.path==='src/control.py'&&item.name==='main')summary='단발 제어 또는 보호 리셋의 CLI 인자를 처리하고 범위 검증·전송·오류 보고를 수행한다.';
   if(f.path==='src/control.py'&&item.name==='send_command')summary='속도·각속도·플래그를 제어 프레임으로 만든 뒤 한 번 전송하고 수신한 응답 문자열을 반환한다.';
   if(f.path==='src/packet_monitor.py'&&item.name==='main')summary='포트와 보드레이트를 받아 수신 전용 루프를 실행하고 유효 프레임의 유형·길이·HEX를 표시한다.';
   if(!summary)throw Error('No summary '+item.name);
   const fid=`${item.type}:${f.path}:${item.name}`;
   nodes.push({id:fid,type:item.type,name:item.name,filePath:f.path,lineRange:[item.startLine,item.endLine],summary,tags:item.type==='class'?['자료-구조','호스트-도구','상태-관리']:['호스트-도구','함수','패킷-처리'],complexity:item.endLine-item.startLine>100?'complex':item.endLine-item.startLine>30?'moderate':'simple'});
   edge(id,fid,'contains');if(exported)edge(id,fid,'exports');
  }
  for(const target of documents[f.path]??[])edge(id,types[target]+':'+target,'documents');
 }
 if(index===5){for(const p of ['src/main.cpp','include/build_features.hpp','src/can_debug.cpp','src/timing_diagnostics.cpp'])edge('config:platformio.ini',types[p]+':'+p,'configures');for(const p of ['src/can_cli.py','src/control.py','src/packet_monitor.py'])edge('config:pyproject.toml',types[p]+':'+p,'configures');}
 if(index===7){
  for(const [owner,name,start,end,summary]of methodData){const id=`function:src/can_cli.py:${owner}.${name}`;nodes.push({id,type:'function',name:owner+'.'+name,filePath:'src/can_cli.py',lineRange:[start,end],summary,tags:['메서드','호스트-도구','상태-처리'],complexity:end-start>80?'complex':end-start>25?'moderate':'simple'});edge('file:src/can_cli.py',id,'contains');edge(`class:src/can_cli.py:${owner}`,id,'contains');}
  const calls=[['main','build_main_parser'],['main','command_from_namespace'],['main','find_teensy_port'],['main','monitor'],['monitor','wait_for_activity'],['monitor','parse_interactive_command'],['monitor','format_packet'],['monitor','PacketStreamParser.feed'],['monitor','BenchmarkStatistics.consume'],['send_command','build_bridge_request'],['send_command','build_benchmark_request'],['build_bridge_request','xor_checksum'],['build_benchmark_request','xor_checksum'],['format_packet','format_timing_packet'],['parse_interactive_command','command_from_namespace']];
  for(const [a,b]of calls)edge('function:src/can_cli.py:'+a,'function:src/can_cli.py:'+b,'calls');
  for(const [a,b]of [['send_command','build_command_packet'],['send_command','send_packet'],['send_reset','build_reset_packet'],['send_reset','send_packet']])edge('function:src/control.py:'+a,'function:src/control.py:'+b,'calls');
  edge('function:src/packet_monitor.py:main','function:src/packet_monitor.py:extract_packets','calls');
 }
 const ids=new Set(nodes.map(n=>n.id));if(ids.size!==nodes.length)throw Error('Duplicate nodes');
 const known=new Set(scan.files.map(f=>types[f.path]+':'+f.path));
 for(const e of edges)if(!ids.has(e.source)||(!ids.has(e.target)&&!known.has(e.target)))throw Error('Unknown edge '+JSON.stringify(e));
 const parts=Math.ceil(Math.max(nodes.length/60,edges.length/120));
 if(parts<=1)fs.writeFileSync(`.ua/intermediate/batch-${index}.json`,JSON.stringify({nodes,edges},null,2));
 else {const files=batch.files.map(f=>f.path).sort();const size=Math.ceil(files.length/parts);for(let k=0;k<parts;k++){const group=new Set(files.slice(k*size,(k+1)*size));const ns=nodes.filter(n=>group.has(n.filePath));const is=new Set(ns.map(n=>n.id));fs.writeFileSync(`.ua/intermediate/batch-${index}-part-${k+1}.json`,JSON.stringify({nodes:ns,edges:edges.filter(e=>is.has(e.source))},null,2));}}
 console.log(JSON.stringify({index,nodes:nodes.length,edges:edges.length,parts,parserSkipped:x.filesSkipped}));
}
