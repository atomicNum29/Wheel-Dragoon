const fs = require('fs');
const path = require('path');
const root = process.cwd();
const batches = JSON.parse(fs.readFileSync('.ua/intermediate/batches.json','utf8')).batches;
const descriptions = {
 'include/control.hpp': ['구동 모드·오류·상태 자료형과 제어 모듈의 공개 인터페이스를 정의한다. USB 명령, RC 구동, 전원 전환 및 CAN 오류 보고가 공유하는 계약이다.', ['제어','인터페이스','상태']],
 'include/rc_io.hpp': ['RC PWM 입력의 범위·중립 데드밴드와 입력 읽기·조향 출력 인터페이스를 정의한다. 채널별 ISR 값을 읽으며 채널 간 원자적 스냅샷은 보장하지 않는다.', ['rc','pwm','인터페이스']],
 'include/steering_config.hpp': ['좌우 서보의 각도–펄스 보정표와 차량 모션 제한값을 정의하고 보정 유효성 검사 및 보간을 제공한다. 소스의 verified 플래그는 이번 분석의 실물 검증 결과가 아니다.', ['조향','보정','설정']],
 'include/vehicle_kinematics.hpp': ['차체 속도·곡률 또는 후륜 중심 PIVOT 각속도를 전륜 조향각과 네 바퀴 RPM 지령으로 변환한다. 조향 범위를 벗어나면 전체 명령을 불가로 표시하고 RPM을 0으로 만든다.', ['운동학','ackermann','pivot']],
 'include/vehicle_motion.hpp': ['기동·제동·조향·정착·주행 상태를 관리하고 네 바퀴 RPM을 공통 비율로 제한·램핑한다. PIVOT은 조향 지령을 즉시 고정각으로 바꾸며 일반 조향 지령 속도 제한을 우회한다.', ['모션','상태기계','속도제한']],
 'src/main.cpp': ['Arduino setup과 loop에서 전원·RC·USB·CAN·모션·진단 처리 순서를 조정한다. 선택적 디버그 및 타이밍 기능은 빌드 플래그에 따라 호출된다.', ['진입점','스케줄링','펌웨어']],
 'src/rc_io.cpp': ['세 RC 채널의 에지 인터럽트에서 펄스폭을 측정하고 10표본 이동평균으로 모드를 선택한다. 보정표를 사용해 전륜 서보 PWM을 출력하며 입력 펄스 만료 검사는 구현하지 않는다.', ['rc','인터럽트','조향']],
 'include/can_tx_schedule.hpp': ['10 ms 명령 주기와 드라이버별 3 ms 응답 구간, 휠 응답 마스크 및 전압 요청 시점을 정의한다. 두 드라이버의 전압은 각각 1 Hz로 요청하고 500 ms 간격을 둔다.', ['can','스케줄링','응답검사']],
 'include/little_endian.hpp': ['CAN·USB 바이트 배열에서 16/32비트 정수를 little-endian으로 읽고 쓰는 공통 변환 함수를 제공한다.', ['직렬화','little-endian','유틸리티']],
 'include/timing_diagnostics.hpp': ['루프 처리 단계와 타이밍 관측 인터페이스를 정의하며 measure 템플릿으로 작업 전후 CPU cycle 차이를 기록한다. 비활성 빌드에서는 작업 실행만 남긴다.', ['계측','빌드조건','인터페이스']],
 'src/can_control.cpp': ['CAN 응답을 ISR 수신 시각 기준으로 검증하고 RPM·엔코더·전압 상태를 갱신한다. 10 ms 주기에서 두 드라이버 명령, 응답 시간초과, 전압 요청과 USB 상태 송신 요청을 조정한다.', ['can','텔레메트리','스케줄링']],
 'src/timing_diagnostics.cpp': ['선택적 타이밍 빌드에서 CPU cycle·RX 간격·휠 응답·시간초과 통계를 수집하고 0x84 USB 진단 패킷으로 내보낸다. USB 여유 공간과 CAN 스케줄을 확인해 진단 송신을 미룬다.', ['계측','진단','usb']]
};
const summaries = {
CommandPacket:'USB 속도 명령의 선속도·각속도 정수 필드와 제어 플래그를 보관한다.',
WheelRpmCommand:'CAN 드라이버로 전달할 LF·LR·RF·RR 바퀴별 RPM 지령을 담는다.',
FaultSnapshot:'반복 송신할 진단 오류와 현재 활성 입력 상태를 구분해 모터 상태 판정에 전달한다.',
Status:'부팅·드라이버 전원·활성화·정지 상태와 직렬 모드 및 바퀴 RPM 목표를 공유한다.',
PwmInput:'속도와 조향 RC 채널의 펄스폭 값을 묶어 제어 루프에 전달한다.',
pwm_duty:'마이크로초 단위 서보 펄스폭을 설정된 주파수와 해상도의 PWM duty 값으로 변환한다.',
CalibrationPoint:'서보 보정표의 한 점을 액추에이터 각도와 마이크로초 펄스폭으로 표현한다.',
ServoCalibration:'서보 핀·유효 범위·각도 반전·단조 보정점과 확인 플래그를 보관한다.',
valid_calibration:'보정점 개수·유한성·단조 증가·중립점과 허용 각도 및 펄스 범위를 검사한다.',
pulse_for_angle:'보정 유효성을 확인한 뒤 차량 각도를 제한·반전하고 인접 보정점 사이를 선형 보간해 펄스폭을 구한다.',
MotionConfig:'차량 기하, RPM 상한, 가감속률, 조향 지령 속도와 기동·정착 대기 시간을 정의한다.',
clamp:'입력 값을 하한과 상한 사이로 제한하는 공통 부동소수점 함수를 제공한다.',
RcCommand:'MANUAL의 선속도·곡률과 PIVOT의 후륜 중심 각속도를 구분해 보관한다.',
Geometry:'휠베이스·전후 윤거·바퀴 반지름을 미터 단위로 보관한다.',
WheelTargets:'두 전륜의 조향각 지령과 네 바퀴 RPM 및 기하학적 실행 가능 여부를 담는다.',
speed_to_rpm:'휠 접선속도를 바퀴 반지름과 분당 초 수를 사용해 RPM으로 환산한다.',
curvature_from_inner_angle:'안쪽 전륜 조향각과 차체 기하로 후륜 중심 경로 곡률을 계산한다.',
inverse:'MANUAL 또는 PIVOT 명령에서 각 바퀴 속도와 전륜 조향각을 계산한다. 입력 또는 조향 범위가 유효하지 않으면 RPM을 0으로 하고 이전 각도를 유지한다.',
MotionController:'보정과 기하 제약을 확인하며 모션 상태 전환, 공통 RPM 제한, 가감속 및 조향 지령 생성을 담당한다.',
calibrated:'좌우 서보 보정표가 모두 유효한지 검사한다.',
valid_configuration:'차량 기하와 RPM·가감속·조향 속도 및 대기 시간 설정이 유한하고 허용 범위인지 확인한다.',
stopped:'생성 중인 모든 바퀴 RPM 지령이 정지 허용 오차 이내인지 판정한다. 실제 바퀴 정지 측정값은 아니다.',
inhibit:'생성 RPM을 즉시 0으로 만들고 준비 상태에 따라 제동 또는 기동 상태로 전환한다.',
update:'기동 지연과 실행 가능성 검사를 거쳐 제동·조향·정착·주행 상태를 진행한다. PIVOT 조향 지령 즉시 반영과 공통 RPM 제한을 적용한다.',
ramp:'각 바퀴의 가감속 제한 중 가장 작은 진행 비율을 공통 적용해 RPM 목표에 접근한다.',
setup:'전원·프로토콜·계측·조향·RC 입력을 초기화한 후 세션과 CAN을 시작하고 부팅 상태를 해제한다.',
loop:'현재 RC 입력을 바탕으로 전원·직렬 수신·CAN 수신·모션·상태 송신·조향 출력 순서로 주기 작업을 실행한다.',
PulseFilter:'각 RC 채널에 독립적인 10표본 순환 버퍼와 합계를 유지해 정수 이동평균을 계산한다.',
push:'가장 오래된 펄스를 새 값으로 교체하고 합계를 갱신해 10표본 평균을 반환한다.',
v_decodePWM:'속도 RC 핀의 에지 간격을 측정해 최대 펄스폭을 넘지 않는 값만 이동평균에 반영한다.',
w_decodePWM:'조향 RC 핀의 에지 간격을 측정해 최대 펄스폭을 넘지 않는 값만 이동평균에 반영한다.',
mode_decodePWM:'모드 채널 펄스폭 범위를 검사하고 필터 출력 구간에 따라 POWERDOWN·STOP·MANUAL·PIVOT·AUTO를 선택한다.',
initialize_inputs:'속도·조향·모드 입력 핀에 CHANGE 인터럽트를 연결한다.',
read_mode:'ISR이 마지막으로 선택한 RC 구동 모드를 반환한다.',
read_pwm:'ISR이 보관한 속도와 조향 펄스폭을 개별 읽기로 반환한다.',
initialize_steering:'유효한 보정표가 있으면 서보 PWM 해상도·주파수를 설정하고 중립각 펄스를 출력한다.',
offset_with_deadband:'입력 펄스폭을 허용 범위로 제한하고 중립 ±50 µs 데드밴드를 제거한 부호 있는 오프셋을 반환한다.',
write_steering:'모션 제어기의 좌우 각도 지령을 서보 보정표와 PWM duty로 변환해 출력한다.',
can_tx_wheel_round_complete:'예상 마스크와 수신 마스크에 네 바퀴가 모두 포함되었는지 확인한다.',
can_tx_response_pair_complete:'지정된 드라이버의 예상 휠 응답 비트가 모두 수신되었는지 확인한다.',
can_tx_response_window_expired:'부호 없는 시간차로 명령 송신 후 3 ms 응답 구간 만료 여부를 판정한다.',
ScheduledVoltageRequest:'예약된 전압 요청의 드라이버 ID·PID·주기 내 송신 시점을 담는다.',
scheduled_voltage_poll:'100주기 슈퍼프레임 중 0번과 50번에서 각각 드라이버 A·B 전압 요청을 예약한다.',
read_u16:'두 바이트를 little-endian 부호 없는 16비트 정수로 조합한다.',
read_i16:'little-endian 16비트 바이트열을 부호 있는 정수로 해석한다.',
read_u32:'네 바이트를 little-endian 부호 없는 32비트 정수로 조합한다.',
read_i32:'little-endian 32비트 바이트열을 부호 있는 정수로 해석한다.',
write_u16:'부호 없는 16비트 값을 하위 바이트부터 버퍼에 기록한다.',
write_i16:'부호 있는 16비트 값을 little-endian 바이트열로 기록한다.',
write_u32:'부호 없는 32비트 값을 하위 바이트부터 버퍼에 기록한다.',
write_i32:'부호 있는 32비트 값을 little-endian 바이트열로 기록한다.',
Md200tDriverCommand:'한 MD200T의 두 모터 채널 RPM과 활성화 여부를 담는다.',
CanTxScheduler:'명령 주기의 시작 시각, 드라이버별 응답 구간, 수신 마스크와 지연 송신 목표를 유지한다.',
status_pending:'USB 시스템 상태 패킷 송신 요청이 대기 중인지 반환한다.',
status_sent:'시스템 상태 패킷 송신 성공 후 대기 플래그를 해제한다.',
begin:'500 kbit/s CAN을 시작하고 초기화 실패 시 안전 송신 오류와 LED 상태를 설정한다.',
drain_rx:'RX polling 간격을 관측하고 수신 큐의 프레임을 처리한다.',
send_torque_off:'주기 스케줄 밖에서 두 드라이버에 즉시 TQ-OFF를 요청하고 송신 실패를 보고한다. 물리적 제동 확인은 수행하지 않는다.',
md200t_response_window_open:'해당 드라이버의 최근 송신이 존재하고 응답 확인 전이며 ISR 수신 시각이 3 ms 이내인지 판정한다.',
md200t_process_rx_frame:'MD200T 응답 ID·길이·응답 구간을 검사하고 바퀴 또는 전압 텔레메트리를 갱신한다. 네 휠 응답이 모이면 USB 상태 송신을 요청한다.',
process_queued_rx:'고정된 최대 횟수만큼 타임스탬프 포함 CAN 프레임을 읽어 전원 상태와 직렬 모드에 맞게 분배한다.',
update_health:'전압 텔레메트리의 유효 기간을 갱신하고 양쪽 값이 유효할 때 저전압 오류를 보고한다.',
invalidate_wheel_telemetry:'누락 휠 비트마스크에 해당하는 RPM·엔코더 텔레메트리의 유효 플래그를 내린다.',
close_driver_response_window:'이미 큐에 도착한 응답을 먼저 처리한 뒤 누락 휠을 무효화하고 시간초과를 보고한다.',
transmit_if_due:'10 ms 주기에 드라이버 A와 B를 순차 송신하며 각 3 ms 응답 구간과 전압 요청 시점을 지킨다. 놓친 주기는 재전송하지 않고 현재 주기로 이동한다.',
reset_session:'수신 텔레메트리와 상태 송신 대기를 초기화하고 CAN 스케줄 기준 시각을 다시 설정한다.'
};
const warnings=[];
for(const index of [1,4]) {
 const batch=batches.find(b=>b.batchIndex===index);
 const extraction=JSON.parse(fs.readFileSync(`.ua/tmp/ua-file-extract-results-${index}.json`,'utf8'));
 if(extraction.filesUnreadable?.length) throw Error('Unreadable source');
 const nodes=[], edges=[], functionByFile=new Map();
 const addEdge=(source,target,type,weight)=>{if(source!==target&&!edges.some(e=>e.source===source&&e.target===target&&e.type===type))edges.push({source,target,type,direction:'forward',weight});};
 for(const file of extraction.results) {
  const [summary,tags]=descriptions[file.path];
  const complexity=file.nonEmptyLines>200?'complex':file.nonEmptyLines>=50?'moderate':'simple';
  nodes.push({id:`file:${file.path}`,type:'file',name:path.basename(file.path),filePath:file.path,summary,tags,complexity});
  if(file.path==='src/can_control.cpp')nodes.at(-1).languageNotes='이번 구조 추출에서는 포인터 반환 함수 telemetry·md200t_find_motor_telemetry와 조건부 diagnostic_output_ready 정의가 누락되어 해당 함수 노드를 제공하지 않는다. 파일 설명은 소스를 읽어 작성했다.';
  if(file.path.includes('timing_diagnostics')) {
   nodes.at(-1).languageNotes='조건부 컴파일과 템플릿 내부 정의는 이번 구조 추출에서 함수·클래스 목록으로 잡히지 않아 파일 단위 설명만 제공한다.';
   warnings.push(`${file.path}: conditional/template declarations missing from structural extraction; file-level semantics retained`);
  }
  const exports=new Set((file.exports||[]).map(e=>e.name));
  const classes=(file.classes||[]).filter(c=>(c.methods?.length||0)>=2||c.endLine-c.startLine+1>=20||exports.has(c.name));
  for(const c of classes) {
   if(!summaries[c.name])throw Error('missing class summary '+c.name);
   const id=`class:${file.path}:${c.name}`;
   nodes.push({id,type:'class',name:c.name,filePath:file.path,lineRange:[c.startLine,c.endLine],summary:summaries[c.name],tags:[...tags.slice(0,2),'자료형'],complexity:c.endLine-c.startLine>=100?'complex':c.endLine-c.startLine>=20?'moderate':'simple'});
   addEdge(`file:${file.path}`,id,'contains',1);
   if(exports.has(c.name))addEdge(`file:${file.path}`,id,'exports',.8);
  }
  const fns=new Map();functionByFile.set(file.path,fns);
  for(const f of file.functions||[]) {
   if(f.endLine-f.startLine+1<10&&!exports.has(f.name))continue;
   if(!summaries[f.name])throw Error('missing function summary '+f.name);
   const owner=classes.find(c=>c.startLine<=f.startLine&&c.endLine>=f.endLine&&(c.methods||[]).includes(f.name));
   const name=owner?`${owner.name}.${f.name}`:f.name;
   const id=`function:${file.path}:${name}`;
   fns.set(f.name,id);
   let summary=summaries[f.name];
   if(owner&&owner.name===f.name)summary=owner.name==='PulseFilter'?'전체 이동평균 표본을 초기 펄스폭으로 채우고 합계를 초기화한다.':owner.name==='MotionController'?'모션 설정과 좌우 서보 보정표를 복사해 제어기를 구성한다.':'선속도·곡률·PIVOT 여부·각속도를 초기값으로 저장한다.';
   nodes.push({id,type:'function',name,filePath:file.path,lineRange:[f.startLine,f.endLine],summary,tags:[...tags.slice(0,2),owner?'메서드':'함수'],complexity:f.endLine-f.startLine>=60?'complex':f.endLine-f.startLine>=20?'moderate':'simple'});
   addEdge(`file:${file.path}`,id,'contains',1);
   if(owner)addEdge(`class:${file.path}:${owner.name}`,id,'contains',1);
   if(exports.has(f.name))addEdge(`file:${file.path}`,id,'exports',.8);
  }
  for(const target of batch.batchImportData[file.path]||[])addEdge(`file:${file.path}`,`file:${target}`,'imports',.7);
 }
 // Resolve parser-observed calls only to known symbols in this batch.
 for(const file of extraction.results) {
  for(const call of file.callGraph||[]) {
   const caller=functionByFile.get(file.path)?.get(call.caller);
   if(!caller)continue;
   const segments=call.callee.split('::');
   const simple=segments.at(-1);
   const namespaces={vehicle:['include/vehicle_kinematics.hpp','include/steering_config.hpp','include/vehicle_motion.hpp'],rc_io:['src/rc_io.cpp'],can_control:['src/can_control.cpp'],little_endian:['include/little_endian.hpp']};
   const permitted=segments.length>1?(namespaces[segments[0]]||[]):null;
   let target=(!permitted||permitted.includes(file.path))?functionByFile.get(file.path)?.get(simple):undefined;
   if(!target) {
    const matches=[];
    for(const [p,fns] of functionByFile)if(p!==file.path&&fns.has(simple)&&(!permitted||permitted.includes(p)))matches.push(fns.get(simple));
    if(matches.length===1)target=matches[0];
   }
   if(target)addEdge(caller,target,'calls',.8);
  }
 }
 const expectedImports=Object.values(batch.batchImportData).reduce((s,a)=>s+a.length,0);
 if(edges.filter(e=>e.type==='imports').length!==expectedImports)throw Error('Import mismatch');
 const ids=new Set(nodes.map(n=>n.id));
 if(ids.size!==nodes.length)throw Error('Duplicate ids');
 for(const e of edges)if(!ids.has(e.source)||(!ids.has(e.target)&&e.type!=='imports'))throw Error('Unresolved edge '+JSON.stringify(e));
 const parts=Math.ceil(Math.max(nodes.length/60,edges.length/120));
 const files=batch.files.map(f=>f.path).sort();
 const chunk=Math.ceil(files.length/parts);
 const outputs=[];
 for(let i=0;i<parts;i++) {
  const fileSet=new Set(files.slice(i*chunk,(i+1)*chunk));
  const partNodes=nodes.filter(n=>fileSet.has(n.filePath));
  const partIds=new Set(partNodes.map(n=>n.id));
  const fragment={nodes:partNodes,edges:edges.filter(e=>partIds.has(e.source))};
  const out=`.ua/intermediate/batch-${index}${parts>1?`-part-${i+1}`:''}.json`;
  fs.writeFileSync(out,JSON.stringify(fragment,null,2));outputs.push(out);
 }
 console.log(JSON.stringify({index,nodes:nodes.length,edges:edges.length,imports:expectedImports,parts,outputs}));
}
warnings.push('src/can_control.cpp: pointer-returning telemetry and md200t_find_motor_telemetry, plus conditional diagnostic_output_ready, absent from bundled extraction.');
fs.writeFileSync('.ua/intermediate/analysis-warnings-1-4.json',JSON.stringify(warnings,null,2));
