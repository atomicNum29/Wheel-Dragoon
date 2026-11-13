const fs = require('fs');
const path = require('path');
const batches = JSON.parse(fs.readFileSync('.ua/intermediate/batches.json','utf8'));
const files = {
 'include/build_features.hpp':['PlatformIO 환경이 지정하는 CAN 디버그와 타이밍 진단 기능 플래그의 기본값을 정의하고, 0 또는 1 이외의 값을 컴파일 오류로 거부한다.',['설정','조건부컴파일','펌웨어']],
 'include/can_control.hpp':['모터 RPM·엔코더·수신 시각과 전압 텔레메트리의 자료형을 정의한다. CAN 수신 처리, 응답 유효성 및 주기 송신을 제어하는 인터페이스를 제공한다.',['can','텔레메트리','인터페이스']],
 'include/can_debug.hpp':['디버그 빌드에서 CAN 브릿지와 반복 요청 측정 인터페이스를 노출한다. 비활성 빌드에는 공통 제어 전환에서 호출할 빈 cancel 함수를 제공한다.',['can','디버그','조건부컴파일']],
 'include/serial_protocol.hpp':['AA 55 USB 프레임의 타입·길이·필드 위치와 허용 명령 범위를 정의한다. 명령 수신, XOR 체크섬 및 상태 송신 인터페이스의 기준이다.',['usb','프로토콜','직렬화']],
 'src/can_debug.cpp':['디버그 빌드에서 USB 요청을 CAN 프레임으로 중계하고 반복 송신의 응답·시간초과·RTT 결과를 수집한다. 브릿지는 첫 수신 프레임을 반환하며, 반복 측정은 ID와 PID 및 수신 시각으로 응답을 구분한다.',['can','브릿지','진단']],
 'src/serial_protocol.cpp':['바이트 상태 기계로 USB 명령과 리셋·디버그 프레임을 분류하고 체크섬과 범위를 검증한다. CAN 텔레메트리와 오류 스냅샷을 0x82 상태 프레임으로 직렬화하며 전체 송신 성공 후 보고 횟수를 소비한다.',['usb','상태기계','직렬화']],
 'include/error_report.hpp':['검출된 오류를 정규 0x82 상태 프레임으로 반복 보고할 횟수 10회를 정의한다. 오류가 재검출되면 남은 보고 횟수를 다시 채우는 정책의 공통 상수다.',['오류보고','설정','상태']],
 'include/flexcan0.hpp':['표준 CAN 프레임과 RX 통계 자료형, 32칸 수신 큐 및 송수신 API를 정의한다. 수신 타임스탬프는 버스 도착 시각이 아니라 ISR이 FIFO를 읽은 시각이다.',['can','인터페이스','수신큐']],
 'include/md200t_can.hpp':['MD200T 드라이버 ID와 요청·모니터·전압 PID를 정의한다. 두 채널 속도 명령, TQ-OFF, PID 요청 및 응답 ID 해석 인터페이스를 제공한다.',['md200t','can','프로토콜']],
 'src/control.cpp':['RC 모드와 전원·시리얼 모드에 따라 구동 허용 상태를 결정하고 운동학 출력과 오류 보고를 관리한다. MANUAL과 PIVOT만 구동하며 AUTO USB 명령은 수신 기록만 유지하고 구동을 차단한다.',['제어','rc','상태기계']],
 'src/flexcan0.cpp':['Teensy 3.2의 FlexCAN0 레지스터를 500 kbit/s로 초기화하고 MB8 송신 및 FIFO 인터럽트 수신을 구현한다. 단일 생산자·소비자 큐로 프레임과 ISR 시각을 전달하고 FIFO·큐 넘침 통계를 누적한다.',['can','인터럽트','수신큐']],
 'src/md200t_can.cpp':['MD200T의 PID 207 속도 명령, PID 174 TQ-OFF 및 PID 4 데이터 요청을 표준 CAN 프레임으로 만든다. 응답은 MID 7을 검증한 뒤 드라이버 ID를 추출한다.',['md200t','can','직렬화']]
};
const summaries = {
 MotorTelemetry:'바퀴의 실제 RPM·엔코더·드라이버 상태와 ISR 수신 시각 및 유효성을 묶는다. 최근 명령 응답 창에서 누락되면 유효 플래그가 해제된다.',
 DriverVoltageTelemetry:'드라이버 전압의 밀리볼트 값과 갱신 시각·유효성을 보관한다. PID 143 데이터의 신선도 판정에 사용한다.',
 Telemetry:'네 바퀴 모터 자료와 두 드라이버 전압 자료를 하나의 CAN 텔레메트리 묶음으로 보관한다.',
 CanFrame:'11비트 표준 CAN ID, 데이터 길이와 최대 8바이트 페이로드를 전달하는 공통 프레임 자료형이다.',
 CanRxStats:'FIFO 넘침·소프트웨어 큐 손실·최대 점유량·최대 큐 대기 시간을 묶는다. 각 필드는 독립적인 원자적 관측값이다.',
 begin:'USB Serial을 지정된 115200 baud 설정으로 초기화한다.',
 parse_complete_command_frame:'XOR 체크섬과 선속도·각속도 범위를 검증한 뒤 CONTROL 모드로 전환하고 명령 수신 시각을 기록한다.',
 parse_complete_reset_frame:'체크섬과 두 바이트 매직 값을 검증한 뒤 MCU 리셋 절차를 호출한다.',
 parse_complete_serial_frame:'길이와 타입 조합에 따라 제어·리셋·브릿지·반복 측정 요청을 분기한다. 제외된 디버그 기능의 프레임도 통째로 소비하고 오류로 보고한다.',
 feed_command_parser:'헤더, 길이, 타입과 나머지 바이트를 순서대로 모으는 수신 상태 기계다. 완성된 프레임을 처리하고 파서를 초기 상태로 되돌린다.',
 receive_commands:'현재 USB 수신 버퍼의 바이트를 모두 읽어 명령 프레임 상태 기계에 전달한다.',
 xor_checksum:'주어진 바이트 배열의 XOR 누적값을 계산하여 USB 프레임 체크섬으로 사용한다.',
 build_system_error_bits:'FaultSnapshot의 오류 플래그를 상태 패킷의 16비트 오류 필드로 변환한다. watchdog에 해당하는 비트는 사용하지 않는다.',
 telemetry_validity_bits:'네 바퀴와 두 전압 자료의 유효 플래그를 상태 프레임의 비트 마스크로 조합한다.',
 send_system_status:'최신 오류와 CAN 텔레메트리를 상태 프레임으로 직렬화한다. USB 전체 쓰기가 성공한 경우에만 순번을 증가시키고 오류 보고 횟수를 소비한다.',
 send_status_if_ready:'CONTROL 모드이고 CAN 상태 송신이 대기 중일 때 상태 패킷을 보낸다. 송신 버퍼 부족 시 대기 플래그를 유지한다.',
 report:'지정 오류의 남은 보고 횟수를 공통 반복 보고 횟수로 다시 채운다.',
 reported:'지정 오류가 아직 보고 대기 상태인지 확인한다.',
 report_safety_tx_failure:'구동 안전 경로의 CAN 송신 실패 보고를 등록한다.',
 initialize_power:'두 드라이버 전원 제어 GPIO를 출력으로 설정하고 HIGH로 초기화한다. 실제 접점 상태를 확인하는 절차는 아니다.',
 finish_boot:'제어 상태의 부팅 중 플래그를 해제한다.',
 initialize_motion:'운동 제어기를 구동 불허·영 입력으로 갱신해 초기 출력 상태를 만든다.',
 initialize_session:'과거 오류 보고와 RX 넘침 보고를 초기화하고 제어 세션 시작 시각 및 CAN 스케줄을 재설정한다.',
 update_power:'POWERDOWN 전환에 따라 전원 GPIO와 기동 대기 상태를 바꾼다. 전원 복귀 후 1초 지연이 지나면 새 제어 세션을 시작한다.',
 consume_error_reports:'남은 오류 보고 횟수를 하나씩 감소시키고 RX 넘침 보고도 소비한다. 정규 상태 패킷의 전체 송신 성공 뒤 호출된다.',
 rpm_to_i16:'유한한 RPM 값을 설정·모터 한계로 제한하고 부호에 맞게 반올림해 int16 명령으로 변환한다.',
 accept_command:'USB 명령 내용과 마지막 유효 수신 시각을 기록한다. 이 함수 자체는 호스트에 구동 권한을 부여하지 않는다.',
 update_drive:'시리얼·전원·RC 모드에 따라 정지 또는 MANUAL·PIVOT 운동 입력을 선택한다. AUTO는 명령 시간초과와 ESTOP 상태만 기록하고 모터 출력을 비활성화한다.',
 reset_mcu:'진단과 주기 구동을 중단하고 가능한 경우 두 드라이버 TQ-OFF를 요청한 뒤 Cortex-M4 소프트웨어 리셋을 수행한다. CAN 실패가 리셋을 무기한 지연시키지는 않는다.',
 enter_control_mode:'진단 작업을 취소하고 필요할 때 BRIDGE에서 CONTROL로 전환하여 제어 세션을 초기화한다.',
 has_driver_fault:'안전 CAN 송신 실패·드라이버 상태 오류·응답 시간초과의 남은 보고 횟수로 구동 차단 오류를 판단한다. 전압 요청 실패는 포함하지 않는다.',
 refresh_errors:'현재 명령 시간초과·ESTOP와 전압 자료 신선도 상태를 반복 오류 보고에 반영한다.',
 fault_snapshot:'즉시 상태와 누적 보고 플래그를 모아 상태 송신에 사용할 오류 스냅샷을 만든다.',
 motor_state:'전원 차단, 오류, ESTOP, 부팅, 명령 시간초과, 비활성 순서로 우선순위를 적용해 모터 상태 코드를 선택한다.',
 wait_until_set:'지정 레지스터 비트가 설정될 때까지 기다리되 마이크로초 제한 시간을 넘으면 실패를 반환한다.',
 wait_until_clear:'지정 레지스터 비트가 지워질 때까지 기다리되 마이크로초 제한 시간을 넘으면 실패를 반환한다.',
 pack_word:'CAN 페이로드 최대 네 바이트를 레지스터 워드 순서로 조합하고 DLC 밖의 바이트는 0으로 채운다.',
 can_rx_isr:'하드웨어 FIFO에서 최대 여섯 프레임을 복사해 타임스탬프와 함께 원자적 링 큐에 게시한다. 큐가 가득 차면 새 프레임을 버리고 넘침 통계를 기록한다.',
 abort_tx_mb:'TX 메일박스가 비활성이면 바로 성공하고, 그렇지 않으면 ABORT 코드와 완료 플래그를 이용해 정리를 시도한다. 실제 하드웨어에서의 중단 동작은 이 분석으로 검증하지 않았다.',
 can_begin:'500 kbit/s만 허용해 FlexCAN0 클록·핀·필터·FIFO·TX 메일박스와 RX 인터럽트를 초기화한다. 레지스터 상태 전환마다 제한 시간으로 실패를 판별한다.',
 can_transmit:'초기화 상태와 표준 ID·DLC를 검증하고 이전 TX 메일박스를 정리한 뒤 한 프레임을 전송한다. 완료 플래그를 제한 시간 내 관측하지 못하면 실패한다.',
 can_receive:'수신 시각이 필요하지 않은 호출자를 위해 시간 정보가 있는 수신 함수를 감싼다.',
 can_receive_timed:'메인 컨텍스트에서 링 큐의 프레임과 ISR 시각을 꺼내고 최대 큐 대기 시간을 갱신한다. timeout_us가 0이면 기다리지 않는다.',
 can_rx_overrun_detected:'FIFO 또는 소프트웨어 큐 손실에 대한 보고가 남아 있는지 확인한다.',
 can_rx_overrun_report_sent:'비교 교환으로 넘침 보고 횟수를 하나 감소시켜 ISR이 새로 등록한 보고와의 경쟁을 처리한다.',
 can_clear_rx_overrun_reports:'새 제어 세션을 위해 과거 RX 넘침 보고 횟수를 지운다.',
 can_is_initialized:'FlexCAN0 초기화가 완료되었는지 반환한다.',
 can_rx_pending:'소프트웨어 큐와 하드웨어 FIFO에 처리할 수신 데이터가 있는지 부작용 없이 관측한다.',
 can_rx_overrun_count:'CAN 초기화 이후 FIFO 넘침 관측 횟수와 소프트웨어 큐 손실 횟수의 합을 반환한다.',
 can_rx_stats:'CAN 수신 손실·최대 점유·최대 대기 시간의 원자적 카운터들을 읽어 통계 구조체로 반환한다.',
 send_frame:'공통 2000 마이크로초 제한 시간으로 CAN 프레임 송신을 호출한다.',
 md200t_set_velocity:'두 모터 채널의 signed RPM을 little-endian으로 넣은 PID 207 프레임을 보낸다. 반환 데이터 요청 필드는 0으로 둔다.',
 md200t_torque_off:'두 채널의 TQ-OFF 활성 비트를 포함한 PID 174 프레임을 보낸다. 이 전송의 성공만으로 물리적 제동을 확인하지는 않는다.',
 md200t_request_pid_data:'PID 4 요청 프레임에 대상 PID를 넣어 송신한다. 응답 해석은 비동기 수신 경로가 담당한다.',
 md200t_decode_response_id:'표준 11비트 ID 범위와 응답 MID 7을 검증하고 하위 바이트에서 드라이버 ID를 꺼낸다.'
};
const excludedPrivate = {
 'src/serial_protocol.cpp': new Set(['CommandFrameParser']),
 'src/flexcan0.cpp': new Set(['RxEntry','mb_code','get_mb_code','elapsed_us','unpack_word']),
 'src/md200t_can.cpp': new Set(['make_standard_id'])
};
const internal = {
 'src/serial_protocol.cpp':new Set(['parse_complete_command_frame','parse_complete_reset_frame','parse_complete_serial_frame','feed_command_parser','build_system_error_bits','telemetry_validity_bits','send_system_status']),
 'src/flexcan0.cpp':new Set(['wait_until_set','wait_until_clear','pack_word','can_rx_isr','abort_tx_mb'])
};
const fragments = new Map();
for(const idx of [2,3]){
 const b=batches.batches.find(b=>b.batchIndex===idx);
 const data=JSON.parse(fs.readFileSync(`.ua/tmp/ua-file-extract-results-${idx}.json`,'utf8'));
 if(data.filesUnreadable.length || data.filesSkipped.length) throw new Error('Unreadable/skipped extraction');
 const nodes=[],edges=[];
 const edge=(source,target,type,weight)=>edges.push({source,target,type,direction:'forward',weight});
 for(const f of data.results){
  const [summary,tags]=files[f.path]; const id='file:'+f.path;
  const node={id,type:'file',name:path.basename(f.path),filePath:f.path,summary,tags,complexity:f.nonEmptyLines>200?'complex':f.nonEmptyLines>=50?'moderate':'simple'};
  if(f.path==='src/can_debug.cpp')node.languageNotes='이 파일은 FIRMWARE_CAN_DEBUG 조건부 컴파일로 둘러싸여 있다. 번들 구조 추출기는 호출 정보만 반환하고 함수·클래스 선언은 반환하지 않아 이 그래프에서는 함수 단위 노드를 생략했다. 파일 요약과 import 관계는 포함한다.';
  if(f.path==='src/control.cpp')node.languageNotes='소스에서 확인한 제어 분기이며 실행·하드웨어 검증 결과는 아니다. 번들 추출기는 참조 반환 접근자와 조건부 enter_bridge_mode 선언을 누락하여 해당 함수 노드는 포함하지 않았다.';
  if(f.path==='src/flexcan0.cpp')node.languageNotes='ISR 단일 생산자와 메인 루프 단일 소비자 사이에서 acquire/release 원자 연산으로 프레임 게시 순서를 제어한다. 레지스터 참조 반환 보조 함수는 번들 추출기에 포함되지 않았다.';
  nodes.push(node);
  const exportNames=new Set((f.exports||[]).map(e=>e.name));
  for(const [type,items] of [['function',f.functions||[]],['class',f.classes||[]]])for(const s of items){
   if(excludedPrivate[f.path]?.has(s.name))continue;
   const exported=exportNames.has(s.name)&&!internal[f.path]?.has(s.name);
   if(!exported && (type==='function'?s.endLine-s.startLine+1<10:((s.methods||[]).length<2&&s.endLine-s.startLine+1<20)))continue;
   if(!summaries[s.name])throw new Error('Missing semantic summary '+f.path+':'+s.name);
   const sid=`${type}:${f.path}:${s.name}`;
   nodes.push({id:sid,type,name:s.name,filePath:f.path,lineRange:[s.startLine,s.endLine],summary:summaries[s.name],tags:type==='class'?['자료형',...tags.slice(0,2)]:['함수',...tags.slice(0,2)],complexity:s.endLine-s.startLine>=50?'complex':s.endLine-s.startLine>=15?'moderate':'simple'});
   edge(id,sid,'contains',1);
   if(exported)edge(id,sid,'exports',0.8);
  }
  for(const target of b.batchImportData[f.path]||[])edge(id,'file:'+target,'imports',0.7);
 }
 fragments.set(idx,{b,data,nodes,edges});
}
const allOwnNodes=[...fragments.values()].flatMap(f=>f.nodes);
const idSet=new Set(allOwnNodes.map(n=>n.id));
const nsPath={control:'src/control.cpp',protocol:'src/serial_protocol.cpp',can_control:'src/can_control.cpp',little_endian:'include/little_endian.hpp'};
for(const [idx,f] of fragments){
 const allowed=new Set(idSet);
 for(const targets of Object.values(f.b.batchImportData))for(const p of targets)allowed.add('file:'+p);
 for(const neighbors of Object.values(f.b.neighborMap))for(const n of neighbors){allowed.add('file:'+n.path);for(const s of n.symbols)allowed.add('function:'+n.path+':'+s);}
 for(const r of f.data.results)for(const c of r.callGraph||[]){
  const source=`function:${r.path}:${c.caller}`;
  if(!idSet.has(source))continue;
  let target;
  if(c.callee.includes('::')){const [ns,name]=c.callee.split('::');if(nsPath[ns])target=`function:${nsPath[ns]}:${name}`;}
  else {const local=`function:${r.path}:${c.callee}`;if(idSet.has(local))target=local;else{const matching=allOwnNodes.filter(n=>n.type==='function'&&n.name===c.callee);if(matching.length===1)target=matching[0].id;}}
  if(target&&target!==source&&allowed.has(target))f.edges.push({source,target,type:'calls',direction:'forward',weight:0.8});
 }
 f.edges=[...new Map(f.edges.map(e=>[`${e.source}|${e.target}|${e.type}`,e])).values()];
 for(const e of f.edges)if(!allowed.has(e.source)||!allowed.has(e.target))throw new Error('Unresolved edge '+JSON.stringify(e));
 const expected=Object.values(f.b.batchImportData).reduce((s,v)=>s+v.length,0);
 if(f.edges.filter(e=>e.type==='imports').length!==expected)throw new Error('Import mismatch');
 const parts=Math.ceil(Math.max(f.nodes.length/60,f.edges.length/120));
 const sorted=f.b.files.map(f=>f.path).sort();
 const chunk=Math.ceil(sorted.length/parts);let written=0;
 for(let offset=0;offset<sorted.length;offset+=chunk){
  const paths=new Set(sorted.slice(offset,offset+chunk));
  const nodes=f.nodes.filter(n=>paths.has(n.filePath));const local=new Set(nodes.map(n=>n.id));
  const edges=f.edges.filter(e=>local.has(e.source));
  const out=`.ua/intermediate/batch-${idx}${parts>1?'-part-'+(++written):''}.json`;
  fs.writeFileSync(out,JSON.stringify({nodes,edges},null,2)+'\n');
  JSON.parse(fs.readFileSync(out,'utf8'));
  console.log(`${out}: ${nodes.length} nodes, ${edges.length} edges`);
 }
 console.log(`batch ${idx}: ${f.nodes.length} nodes, ${f.edges.length} edges, ${expected} imports`);
}
