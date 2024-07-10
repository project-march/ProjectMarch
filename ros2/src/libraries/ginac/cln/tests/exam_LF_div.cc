#include "exam.h"
#include <cln/lfloat.h>
#include <cln/lfloat_io.h>

static div_test lfloat_div_tests[] = {

{ "0.8476517865511829377L0", "0.14598720922015648169L0",
  "5.8063428370144327317L0" },

{ "-0.16515392772872533974L0", "0.2885771921352848653L0",
  "-0.5723041606534907598L0" },

{ "-0.25791761734650428572L0", "-0.17472849542471660309L0",
  "1.4761050664322265015L0" },

{ "-0.9418668871216534004L0", "-0.25801526180943099573L0",
  "3.6504309106230792821L0" },

{ "-0.25726582509610465451L0", "0.7704327058756196045L0",
  "-0.33392381078074095957L0" },

{ "-0.021409432992321506645L0", "-0.44293479613874918959L0",
  "0.048335405524597819813L0" },

{ "-0.26771090178828336857L0", "-7.7011892538310270067L9",
  "3.4762280599078659542L-11" },

{ "0.82867609196336006595L0", "-3.0042819216966844948L9",
  "-2.7583166745395210866L-10" },

{ "0.40732354689187331287L0", "-7.559832309976744222L9",
  "-5.387997116739304149L-11" },

{ "-0.53349543673778000914L0", "-5.104278761341346705L8",
  "1.045192595628502556L-9" },

{ "0.17669669311850475256L0", "-9.181879875841464834L9",
  "-1.9244064996255633173L-11" },

{ "-0.38525727576606363245L0", "-4.8936643582468263693L9",
  "7.872572525674472248L-11" },

{ "0.028580272067667963345L0", "-7.9841173999044091L-11",
  "-3.5796407587907142282L8" },

{ "0.72167998280372380157L0", "-3.6437273419914776347L-12",
  "-1.980609181391958688L11" },

{ "-0.5863461999919387516L0", "7.881986348526466578L-11",
  "-7.4390664239294943926L9" },

{ "0.54541403791059564303L0", "-2.6107257402815120583L-11",
  "-2.089128051619026705L10" },

{ "0.7985324354238058011L0", "9.752737902348257611L-12",
  "8.187777047012979847L10" },

{ "0.14104671220162837288L0", "-1.679932803469743255L-11",
  "-8.3959734526470136372L9" },

{ "0.84226961154302812054L0", "-3.7790325979515268584L19",
  "-2.2287968936801211454L-20" },

{ "-0.17023320737807742781L0", "-7.0544793122604881768L19",
  "2.4131222141684768152L-21" },

{ "0.51147038234753495475L0", "7.2890488826322506176L19",
  "7.01697012303244035L-21" },

{ "0.15424860911694467965L0", "-9.2121691156562017736L19",
  "-1.6744005367291526718L-21" },

{ "0.18043991101271504866L0", "-1.5135729370916590423L19",
  "-1.1921454631676461953L-20" },

{ "-0.8669749687756526617L0", "8.7133495928438747096L19",
  "-9.949961946754488136L-21" },

{ "-0.6448505560111598971L0", "3.636469578348857873L-21",
  "-1.7732873659951112376L20" },

{ "-0.81857582399766609004L0", "5.2916132942068490006L-21",
  "-1.546930545536701092L20" },

{ "0.77524450276763022L0", "-7.652595302708246449L-21",
  "-1.0130478250865714831L20" },

{ "0.627858729575384142L0", "9.627326573065363056L-21",
  "6.5216311590796329432L19" },

{ "-0.42943946308533227006L0", "-2.2414950519882640498L-21",
  "1.9158617490786266339L20" },

{ "-0.34220208112358558038L0", "-7.4545803279812700505L-21",
  "4.5904942473972275588L19" },

{ "5.560943842255079481L9", "-0.55841023848214400133L0",
  "-9.958527725728472542L9" },

{ "-8.661678305761957921L9", "-0.87958882986448744696L0",
  "9.847417351919312785L9" },

{ "3.6954900583503502368L9", "-0.36989453222048823558L0",
  "-9.9906587863470431315L9" },

{ "6.740385471899914443L8", "-0.2745720588185960522L0",
  "-2.454869406924301959L9" },

{ "-5.1381279403866914758L9", "-0.32555782051482221485L0",
  "1.578253574821668073L10" },

{ "-3.2065087686035281697L9", "0.50505516522796299416L0",
  "-6.3488287802308291444L9" },

{ "-7.7979994067331648055L9", "-6.4459990751639263853L9",
  "1.2097425574847536075L0" },

{ "-4.7272619195621447717L9", "-3.825695015629283172L8",
  "12.356609453314103588L0" },

{ "-3.5376744034596315073L9", "2.7483444719369282795L9",
  "-1.2872019645217230068L0" },

{ "-2.2400216393287578975L9", "-3.7058330823204350567L9",
  "0.6044583200509807153L0" },

{ "3.0621742151056386386L9", "-8.846101104908494769L9",
  "-0.34616088814613589822L0" },

{ "7.5149875074517868906L9", "4.423024956398348232L9",
  "1.6990606161017937287L0" },

{ "7.6970261502618782055L9", "-9.7716080626747355186L-11",
  "-7.876928854384493259L19" },

{ "-8.725835744855911806L8", "8.409822932470646079L-11",
  "-1.0375766309139670758L19" },

{ "5.820797723708174118L9", "-2.882166534035175912L-11",
  "-2.0195910454760464445L20" },

{ "1.207852991950790034L9", "5.840354579417081103L-11",
  "2.068115857567237665L19" },

{ "3.1046967393071541823L9", "-5.5642977043818474125L-11",
  "-5.5796740294147564416L19" },

{ "4.392532668212736406L9", "-7.535498815249885942L-11",
  "-5.8291199771983175508L19" },

{ "1.3280881496906639524L9", "-1.766515912740190632L19",
  "-7.518121632035316941L-11" },

{ "1.4277961930808139626L9", "-8.986506745304867108L19",
  "-1.5888222571321021114L-11" },

{ "-7.9134656119390343763L9", "-3.4095849226963530828L19",
  "2.3209469162249057589L-10" },

{ "-8.7882725472722691335L9", "-5.186325400713441962L19",
  "1.6945085138821670647L-10" },

{ "3.8930727351090315925L9", "-7.3980221641298868864L19",
  "-5.2623155875161027887L-11" },

{ "9.998404421166073569L9", "-8.1317115085820412065L18",
  "-1.2295571984586471158L-9" },

{ "4.623792381028250544L9", "6.996281129080973142L-21",
  "6.6089287947690416075L29" },

{ "8.472924939037688662L9", "-4.3460987737519244214L-22",
  "-1.9495472560839050376L31" },

{ "1.9551595642940545935L9", "-7.5324972045717692564L-21",
  "-2.5956326450508222435L29" },

{ "-8.5478772651240992225L9", "-2.4212066230883777513L-21",
  "3.5304204042779411337L30" },

{ "6.881700625121950854L9", "-8.203099619911879591L-21",
  "-8.389146717684109215L29" },

{ "6.097099876947129031L9", "-4.76850418677518328L-21",
  "-1.2786189627046213921L30" },

{ "-5.0358061432469478737L-11", "-0.6780392915138573621L0",
  "7.427012278305451381L-11" },

{ "-1.49762284327640383L-11", "0.15227257119521089694L0",
  "-9.835145171066142436L-11" },

{ "-7.1678035946969115934L-11", "0.75360681415553320054L0",
  "-9.511330656861051013L-11" },

{ "4.583894304978394541L-12", "0.68934670181533335835L0",
  "6.6496210004445016106L-12" },

{ "-3.8885547056166489716L-12", "-0.010643810658165133798L0",
  "3.6533482513930678043L-10" },

{ "9.49880444227161124L-11", "-0.122629749019578004226L0",
  "-7.745921783428843474L-10" },

{ "2.1990660545226500317L-11", "-1.4161745224867819854L9",
  "-1.552821364602098501L-20" },

{ "9.951737846856727225L-11", "-6.0164204240154494783L9",
  "-1.6540961477912788069L-20" },

{ "-5.873282338412930208L-11", "2.3788798751415933107L9",
  "-2.4689276662460085614L-20" },

{ "2.2209512664584027642L-11", "5.1944018613813348683L9",
  "4.2756631576205975403L-21" },

{ "-6.722318330051584872L-11", "6.7936247801916195024L9",
  "-9.895039169151724367L-21" },

{ "-7.528877773200399613L-12", "-9.535757813603057891L9",
  "7.89541630604357328L-22" },

{ "-2.0857643618410047184L-11", "2.701544718271986855L-11",
  "-0.77206360780699598463L0" },

{ "2.5510439626733908612L-11", "1.6734405694946451074L-11",
  "1.5244305708709866576L0" },

{ "3.048460642905138835L-11", "5.1568899955161432057L-11",
  "0.5911432366321058725L0" },

{ "9.876491787625061464L-12", "8.667781903943973216L-11",
  "0.113944858062604306884L0" },

{ "1.1166642175553123016L-11", "-7.759981600144040302L-11",
  "-0.14390036923986841014L0" },

{ "-2.7282824760136843772L-11", "-9.160281916489131182L-11",
  "0.2978382653379466574L0" },

{ "-3.1587174777348029438L-11", "-4.9090150171793744104L19",
  "6.4345239659701453497L-31" },

{ "-4.512784364891002838L-11", "5.9600731551720265308L19",
  "-7.571692909465218857L-31" },

{ "-1.431681316436341718L-11", "-4.22349605246125618L19",
  "3.3898014788057508284L-31" },

{ "-6.719040537613210677L-11", "-4.545488183802435408L19",
  "1.47817797911258332435L-30" },

{ "2.5092238442261623676L-11", "3.3004591427193857704L19",
  "7.602650830449936487L-31" },

{ "-6.198495042920933878L-12", "1.8747110273916984954L19",
  "-3.306373596972410786L-31" },

{ "8.3326031863190006605L-11", "6.3679312781687389584L-21",
  "1.3085259281747860217L10" },

{ "2.228308172351851791L-11", "-4.6204647093882084617L-22",
  "-4.822692764700068564L10" },

{ "9.7676469315043868665L-11", "-6.6370355345926113967L-21",
  "-1.4716882078745621587L10" },

{ "-8.9713798012161717115L-11", "-3.669192301028840519L-21",
  "2.445055768459069116L10" },

{ "7.214258511983827207L-11", "-1.5195990661514104949L-21",
  "-4.7474749574931692373L10" },

{ "1.4822028144092954099L-12", "2.269595713994387529L-21",
  "6.530690929974856047L8" },

{ "-4.6354687290142894644L19", "0.032331325634476806982L0",
  "-1.4337391486574910728L21" },

{ "-2.389352438897577318L19", "0.8660312577952003013L0",
  "-2.7589678979723536864L19" },

{ "-2.4109458405628950432L19", "0.26688102636777617506L0",
  "-9.0337851040803631776L19" },

{ "8.961066349333904704L19", "-0.66178143682771294813L0",
  "-1.35408245844568974384L20" },

{ "6.6419769467305502364L19", "-0.8456142496793601811L0",
  "-7.854618047471472417L19" },

{ "3.7389082257286159308L19", "0.56261989685796304976L0",
  "6.645531462021022254L19" },

{ "7.814283695666500025L19", "-4.6620013293904720047L9",
  "-1.6761650509199167363L10" },

{ "6.6434731737611309404L19", "-2.858805223329136325L9",
  "-2.323863521567472329L10" },

{ "-1.3409334390407788129L19", "6.1497605350647401055L9",
  "-2.1804644772671013651L9" },

{ "7.0858597943822241668L19", "-2.58410378455919273L9",
  "-2.7420956684179617314L10" },

{ "-6.6455998228898640428L19", "-7.7545004942277582046L9",
  "8.569990843171226794L9" },

{ "2.9602494058183339616L19", "-5.7169856186590364077L9",
  "-5.1779899465842692843L9" },

{ "-6.698311323164055808L19", "-6.553232827426109497L-11",
  "1.02213846197113193186L30" },

{ "-7.554561034956199475L19", "6.4764910162760040714L-11",
  "-1.1664589692120175174L30" },

{ "6.7796490729162210612L19", "9.9915237995070190003L-11",
  "6.785400514434773617L29" },

{ "-6.9067747658009050975L18", "-2.5761632749585983355L-11",
  "2.681031452058062687L29" },

{ "1.629413698021581386L19", "-8.612780517302459862L-11",
  "-1.8918555915226283107L29" },

{ "8.8732593909692189064L19", "-4.0536919536865455935L-12",
  "-2.1889328277398133904L31" },

{ "4.8426213700963381164L19", "7.883038261101094331L19",
  "0.61430900240485778846L0" },

{ "-5.2968355222513127376L19", "1.5071497411718048594L19",
  "-3.5144719715328600349L0" },

{ "-6.2610887651422622925L18", "1.0358424497888766788L19",
  "-0.60444411854509194816L0" },

{ "-2.4670994205369878408L19", "6.9747461294856021948L19",
  "-0.3537188844920639511L0" },

{ "6.9460731069354980812L19", "3.1486762233902586798L19",
  "2.2060296499639734035L0" },

{ "8.8228286449463631936L19", "6.7354354317536527728L19",
  "1.3099121406987093833L0" },

{ "3.2098388728662261428L19", "-2.6305167886064038438L-21",
  "-1.2202312818412901165L40" },

{ "-7.144492994496515916L19", "-2.0335028635662185032L-21",
  "3.5133921483478965099L40" },

{ "-6.3695870249569899508L19", "1.9319318539671607067L-21",
  "-3.2970039869042198792L40" },

{ "-5.4056057590545112688L19", "6.6371220252553042967L-21",
  "-8.144502599899959829L39" },

{ "-4.5534797093596626272L19", "9.223324048915255164L-21",
  "-4.9369182793650108047L39" },

{ "3.9206183123968272208L19", "-1.6559061178638737343L-21",
  "-2.3676573629998072004L40" },

{ "-8.768637785982664131L-21", "-0.18184176456694917492L0",
  "4.8221253279547290195L-20" },

{ "2.6823352573966718016L-21", "-0.55524799130252431824L0",
  "-4.830877912956219511L-21" },

{ "-4.0350541003620172524L-21", "0.27000304046926068644L0",
  "-1.4944476526446376082L-20" },

{ "6.332356861830292899L-21", "0.65544003241974460534L0",
  "9.6612299350294242524L-21" },

{ "3.5603120340723305693L-21", "-0.124100556644984066966L0",
  "-2.86889288035778711L-20" },

{ "5.5961094005028721084L-21", "0.47201702367299511838L0",
  "1.18557363820414998006L-20" },

{ "1.7187188076305931646L-21", "8.3685668129856246863L9",
  "2.0537791548292745125L-31" },

{ "-2.7220241842791803757L-21", "2.2892422122227956846L9",
  "-1.1890503196846804849L-30" },

{ "-6.028203796038167925L-21", "-5.415224539645905615L9",
  "1.1131955382282900156L-30" },

{ "6.6310444174308960725L-21", "9.461342958972558645L9",
  "7.0085657460946591684L-31" },

{ "-8.8033709586752979635L-21", "2.8098765759657792274L9",
  "-3.1330098389284241575L-30" },

{ "-3.4027974212452472475L-21", "6.219628754500815959L8",
  "-5.4710619484849846614L-30" },

{ "8.388977931970215088L-21", "2.8213325814913435694L-11",
  "2.9734097947204223302L-10" },

{ "-9.3496400462478483586L-21", "-9.381494249123695733L-11",
  "9.966045704415559596L-11" },

{ "-6.936639418470504025L-21", "5.6618206553549859367L-11",
  "-1.2251605694909792675L-10" },

{ "-2.3667892015182913211L-21", "-7.1545639578577691874L-11",
  "3.3080830857887236957L-11" },

{ "-9.576766108065157562L-21", "-6.4350290609494113365L-11",
  "1.4882242204905008798L-10" },

{ "-2.5955914883538434001L-22", "5.8091383646322322124L-11",
  "-4.4681178609147595716L-12" },

{ "-2.9619491950657497217L-21", "-5.3730670726011346488L19",
  "5.512585558757694777L-41" },

{ "2.5726455340193007026L-22", "3.0037766865540527038L19",
  "8.564703047118500122L-42" },

{ "-2.8277317971003367574L-21", "-4.4068191966128705184L19",
  "6.4167184332721487087L-41" },

{ "7.503784949731224261L-21", "5.9540210967055505192L19",
  "1.2602886062804146604L-40" },

{ "1.4876876016319254574L-22", "8.6818746213386148185L18",
  "1.7135557313571827969L-41" },

{ "2.699544264870480357L-21", "3.6796341400587007856L19",
  "7.3364474893892979093L-41" },

{ "-7.285812539718203862L-21", "5.700589904684711396L-21",
  "-1.2780804551000530294L0" },

{ "3.6474102791520560028L-21", "-6.343773677116707765L-21",
  "-0.574959080319812269L0" },

{ "-4.2510720089860863712L-21", "-8.281980897162330288L-21",
  "0.51329169455614642465L0" },

{ "5.770684998505203844L-21", "6.5700291863604419324L-21",
  "0.8783347584642853315L0" },

{ "-4.8018196973750014744L-21", "-7.3250029580209059804L-21",
  "0.6555382605159211192L0" },

{ "-3.9261100835261094614L-21", "-8.986577968334144672L-21",
  "0.436885997913830856L0" },

};
