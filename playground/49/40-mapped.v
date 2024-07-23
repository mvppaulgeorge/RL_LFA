// Benchmark "adder" written by ABC on Thu Jul 18 14:55:46 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n131, new_n132,
    new_n133, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n150, new_n151, new_n152, new_n153, new_n154, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n317,
    new_n320, new_n321, new_n322, new_n323, new_n325, new_n326, new_n327;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor022aa1n08x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  and002aa1n12x5               g003(.a(\b[0] ), .b(\a[1] ), .o(new_n99));
  oaoi03aa1n12x5               g004(.a(\a[2] ), .b(\b[1] ), .c(new_n99), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nand42aa1n20x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nor042aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nand42aa1n06x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nano23aa1n03x5               g009(.a(new_n101), .b(new_n103), .c(new_n104), .d(new_n102), .out0(new_n105));
  tech160nm_fiao0012aa1n02p5x5 g010(.a(new_n101), .b(new_n103), .c(new_n102), .o(new_n106));
  tech160nm_fiaoi012aa1n04x5   g011(.a(new_n106), .b(new_n105), .c(new_n100), .o1(new_n107));
  norp02aa1n02x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  nand02aa1n08x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nor002aa1d32x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nand02aa1n06x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nano23aa1n02x4               g016(.a(new_n108), .b(new_n110), .c(new_n111), .d(new_n109), .out0(new_n112));
  xorc02aa1n02x5               g017(.a(\a[5] ), .b(\b[4] ), .out0(new_n113));
  xorc02aa1n02x5               g018(.a(\a[8] ), .b(\b[7] ), .out0(new_n114));
  nand23aa1n03x5               g019(.a(new_n112), .b(new_n113), .c(new_n114), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  oai022aa1n02x5               g021(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n117));
  nano22aa1n02x4               g022(.a(new_n110), .b(new_n109), .c(new_n111), .out0(new_n118));
  oai022aa1n02x5               g023(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n119));
  aoai13aa1n09x5               g024(.a(new_n116), .b(new_n119), .c(new_n118), .d(new_n117), .o1(new_n120));
  oai012aa1n12x5               g025(.a(new_n120), .b(new_n107), .c(new_n115), .o1(new_n121));
  nanp02aa1n02x5               g026(.a(\b[8] ), .b(\a[9] ), .o1(new_n122));
  nanb02aa1n02x5               g027(.a(new_n97), .b(new_n122), .out0(new_n123));
  inv000aa1d42x5               g028(.a(new_n123), .o1(new_n124));
  nanp02aa1n02x5               g029(.a(new_n121), .b(new_n124), .o1(new_n125));
  norp02aa1n02x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nand02aa1n03x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  norb02aa1n02x5               g032(.a(new_n127), .b(new_n126), .out0(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n128), .b(new_n125), .c(new_n98), .out0(\s[10] ));
  norp02aa1n04x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  tech160nm_finand02aa1n03p5x5 g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nanb02aa1n02x5               g036(.a(new_n130), .b(new_n131), .out0(new_n132));
  nona22aa1n02x4               g037(.a(new_n125), .b(new_n126), .c(new_n97), .out0(new_n133));
  xnbna2aa1n03x5               g038(.a(new_n132), .b(new_n133), .c(new_n127), .out0(\s[11] ));
  nano22aa1n02x4               g039(.a(new_n130), .b(new_n127), .c(new_n131), .out0(new_n135));
  tech160nm_fiao0012aa1n02p5x5 g040(.a(new_n130), .b(new_n133), .c(new_n135), .o(new_n136));
  norp02aa1n04x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nand42aa1n03x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(new_n139));
  aoi113aa1n02x5               g044(.a(new_n130), .b(new_n139), .c(new_n133), .d(new_n131), .e(new_n127), .o1(new_n140));
  aoi012aa1n02x5               g045(.a(new_n140), .b(new_n136), .c(new_n139), .o1(\s[12] ));
  nona23aa1n09x5               g046(.a(new_n138), .b(new_n131), .c(new_n130), .d(new_n137), .out0(new_n142));
  nona23aa1n03x5               g047(.a(new_n127), .b(new_n122), .c(new_n97), .d(new_n126), .out0(new_n143));
  nor042aa1n06x5               g048(.a(new_n143), .b(new_n142), .o1(new_n144));
  oai012aa1n02x5               g049(.a(new_n127), .b(new_n126), .c(new_n97), .o1(new_n145));
  ao0012aa1n03x7               g050(.a(new_n137), .b(new_n130), .c(new_n138), .o(new_n146));
  oabi12aa1n02x5               g051(.a(new_n146), .b(new_n142), .c(new_n145), .out0(new_n147));
  tech160nm_fiao0012aa1n02p5x5 g052(.a(new_n147), .b(new_n121), .c(new_n144), .o(new_n148));
  xorb03aa1n02x5               g053(.a(new_n148), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1n02x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  nanp02aa1n02x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  aoi012aa1n02x5               g056(.a(new_n150), .b(new_n148), .c(new_n151), .o1(new_n152));
  norp02aa1n03x5               g057(.a(\b[13] ), .b(\a[14] ), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(\b[13] ), .b(\a[14] ), .o1(new_n154));
  nanb02aa1n02x5               g059(.a(new_n153), .b(new_n154), .out0(new_n155));
  inv000aa1d42x5               g060(.a(new_n155), .o1(new_n156));
  aoi112aa1n02x5               g061(.a(new_n150), .b(new_n156), .c(new_n148), .d(new_n151), .o1(new_n157));
  oab012aa1n02x4               g062(.a(new_n157), .b(new_n152), .c(new_n155), .out0(\s[14] ));
  nona23aa1n06x5               g063(.a(new_n154), .b(new_n151), .c(new_n150), .d(new_n153), .out0(new_n159));
  inv020aa1n02x5               g064(.a(new_n159), .o1(new_n160));
  aoai13aa1n06x5               g065(.a(new_n160), .b(new_n147), .c(new_n121), .d(new_n144), .o1(new_n161));
  oai012aa1n02x5               g066(.a(new_n154), .b(new_n153), .c(new_n150), .o1(new_n162));
  nor022aa1n16x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  nanp02aa1n04x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  norb02aa1n03x5               g069(.a(new_n164), .b(new_n163), .out0(new_n165));
  xnbna2aa1n03x5               g070(.a(new_n165), .b(new_n161), .c(new_n162), .out0(\s[15] ));
  aob012aa1n03x5               g071(.a(new_n165), .b(new_n161), .c(new_n162), .out0(new_n167));
  inv000aa1d42x5               g072(.a(new_n163), .o1(new_n168));
  inv000aa1d42x5               g073(.a(new_n165), .o1(new_n169));
  aoai13aa1n02x5               g074(.a(new_n168), .b(new_n169), .c(new_n161), .d(new_n162), .o1(new_n170));
  nor022aa1n06x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  nanp02aa1n04x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  aoib12aa1n02x5               g078(.a(new_n163), .b(new_n172), .c(new_n171), .out0(new_n174));
  aoi022aa1n02x7               g079(.a(new_n170), .b(new_n173), .c(new_n167), .d(new_n174), .o1(\s[16] ));
  nona23aa1d18x5               g080(.a(new_n172), .b(new_n164), .c(new_n163), .d(new_n171), .out0(new_n176));
  nona22aa1n09x5               g081(.a(new_n144), .b(new_n159), .c(new_n176), .out0(new_n177));
  nanb02aa1n06x5               g082(.a(new_n177), .b(new_n121), .out0(new_n178));
  inv000aa1d42x5               g083(.a(new_n176), .o1(new_n179));
  nano23aa1n03x5               g084(.a(new_n130), .b(new_n137), .c(new_n138), .d(new_n131), .out0(new_n180));
  tech160nm_fioaoi03aa1n03p5x5 g085(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n181));
  aoai13aa1n06x5               g086(.a(new_n160), .b(new_n146), .c(new_n180), .d(new_n181), .o1(new_n182));
  aob012aa1n03x5               g087(.a(new_n179), .b(new_n182), .c(new_n162), .out0(new_n183));
  aoi012aa1n02x5               g088(.a(new_n171), .b(new_n163), .c(new_n172), .o1(new_n184));
  nand23aa1n06x5               g089(.a(new_n178), .b(new_n183), .c(new_n184), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g091(.a(\a[17] ), .o1(new_n187));
  nanb02aa1n02x5               g092(.a(\b[16] ), .b(new_n187), .out0(new_n188));
  oaoi13aa1n12x5               g093(.a(new_n177), .b(new_n120), .c(new_n107), .d(new_n115), .o1(new_n189));
  aoai13aa1n06x5               g094(.a(new_n184), .b(new_n176), .c(new_n182), .d(new_n162), .o1(new_n190));
  xorc02aa1n02x5               g095(.a(\a[17] ), .b(\b[16] ), .out0(new_n191));
  oai012aa1n02x5               g096(.a(new_n191), .b(new_n190), .c(new_n189), .o1(new_n192));
  tech160nm_fixorc02aa1n03p5x5 g097(.a(\a[18] ), .b(\b[17] ), .out0(new_n193));
  xnbna2aa1n03x5               g098(.a(new_n193), .b(new_n192), .c(new_n188), .out0(\s[18] ));
  inv000aa1d42x5               g099(.a(\a[18] ), .o1(new_n195));
  xroi22aa1d04x5               g100(.a(new_n187), .b(\b[16] ), .c(new_n195), .d(\b[17] ), .out0(new_n196));
  oaih12aa1n02x5               g101(.a(new_n196), .b(new_n190), .c(new_n189), .o1(new_n197));
  oai022aa1n02x7               g102(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n198));
  oaib12aa1n09x5               g103(.a(new_n198), .b(new_n195), .c(\b[17] ), .out0(new_n199));
  nor042aa1n09x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nand22aa1n04x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  nanb02aa1n02x5               g106(.a(new_n200), .b(new_n201), .out0(new_n202));
  inv000aa1d42x5               g107(.a(new_n202), .o1(new_n203));
  xnbna2aa1n03x5               g108(.a(new_n203), .b(new_n197), .c(new_n199), .out0(\s[19] ));
  xnrc02aa1n02x5               g109(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  oaoi03aa1n02x5               g110(.a(\a[18] ), .b(\b[17] ), .c(new_n188), .o1(new_n206));
  aoai13aa1n02x5               g111(.a(new_n203), .b(new_n206), .c(new_n185), .d(new_n196), .o1(new_n207));
  inv040aa1n03x5               g112(.a(new_n200), .o1(new_n208));
  aoai13aa1n02x5               g113(.a(new_n208), .b(new_n202), .c(new_n197), .d(new_n199), .o1(new_n209));
  nor002aa1n03x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  nand22aa1n04x5               g115(.a(\b[19] ), .b(\a[20] ), .o1(new_n211));
  norb02aa1n02x5               g116(.a(new_n211), .b(new_n210), .out0(new_n212));
  aoib12aa1n02x5               g117(.a(new_n200), .b(new_n211), .c(new_n210), .out0(new_n213));
  aoi022aa1n03x5               g118(.a(new_n209), .b(new_n212), .c(new_n207), .d(new_n213), .o1(\s[20] ));
  nano23aa1n06x5               g119(.a(new_n200), .b(new_n210), .c(new_n211), .d(new_n201), .out0(new_n215));
  nand23aa1n04x5               g120(.a(new_n215), .b(new_n191), .c(new_n193), .o1(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  oaih12aa1n02x5               g122(.a(new_n217), .b(new_n190), .c(new_n189), .o1(new_n218));
  nona23aa1n09x5               g123(.a(new_n211), .b(new_n201), .c(new_n200), .d(new_n210), .out0(new_n219));
  oaoi03aa1n06x5               g124(.a(\a[20] ), .b(\b[19] ), .c(new_n208), .o1(new_n220));
  oabi12aa1n18x5               g125(.a(new_n220), .b(new_n219), .c(new_n199), .out0(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  xorc02aa1n02x5               g127(.a(\a[21] ), .b(\b[20] ), .out0(new_n223));
  xnbna2aa1n03x5               g128(.a(new_n223), .b(new_n218), .c(new_n222), .out0(\s[21] ));
  aoai13aa1n03x5               g129(.a(new_n223), .b(new_n221), .c(new_n185), .d(new_n217), .o1(new_n225));
  nor002aa1n03x5               g130(.a(\b[20] ), .b(\a[21] ), .o1(new_n226));
  inv040aa1n02x5               g131(.a(new_n226), .o1(new_n227));
  inv000aa1d42x5               g132(.a(new_n223), .o1(new_n228));
  aoai13aa1n02x5               g133(.a(new_n227), .b(new_n228), .c(new_n218), .d(new_n222), .o1(new_n229));
  xorc02aa1n02x5               g134(.a(\a[22] ), .b(\b[21] ), .out0(new_n230));
  norp02aa1n02x5               g135(.a(new_n230), .b(new_n226), .o1(new_n231));
  aoi022aa1n03x5               g136(.a(new_n229), .b(new_n230), .c(new_n225), .d(new_n231), .o1(\s[22] ));
  nanp02aa1n02x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  xnrc02aa1n02x5               g138(.a(\b[21] ), .b(\a[22] ), .out0(new_n234));
  nano22aa1n03x7               g139(.a(new_n234), .b(new_n227), .c(new_n233), .out0(new_n235));
  norb02aa1n03x5               g140(.a(new_n235), .b(new_n216), .out0(new_n236));
  oai012aa1n06x5               g141(.a(new_n236), .b(new_n190), .c(new_n189), .o1(new_n237));
  oao003aa1n12x5               g142(.a(\a[22] ), .b(\b[21] ), .c(new_n227), .carry(new_n238));
  inv000aa1d42x5               g143(.a(new_n238), .o1(new_n239));
  aoi012aa1n02x5               g144(.a(new_n239), .b(new_n221), .c(new_n235), .o1(new_n240));
  nand42aa1n02x5               g145(.a(new_n237), .b(new_n240), .o1(new_n241));
  xorc02aa1n12x5               g146(.a(\a[23] ), .b(\b[22] ), .out0(new_n242));
  aoi112aa1n02x5               g147(.a(new_n242), .b(new_n239), .c(new_n221), .d(new_n235), .o1(new_n243));
  aoi022aa1n02x5               g148(.a(new_n241), .b(new_n242), .c(new_n237), .d(new_n243), .o1(\s[23] ));
  nanp02aa1n03x5               g149(.a(new_n241), .b(new_n242), .o1(new_n245));
  nor042aa1n03x5               g150(.a(\b[22] ), .b(\a[23] ), .o1(new_n246));
  inv000aa1d42x5               g151(.a(new_n246), .o1(new_n247));
  inv000aa1d42x5               g152(.a(new_n242), .o1(new_n248));
  aoai13aa1n03x5               g153(.a(new_n247), .b(new_n248), .c(new_n237), .d(new_n240), .o1(new_n249));
  tech160nm_fixorc02aa1n04x5   g154(.a(\a[24] ), .b(\b[23] ), .out0(new_n250));
  norp02aa1n02x5               g155(.a(new_n250), .b(new_n246), .o1(new_n251));
  aoi022aa1n03x5               g156(.a(new_n249), .b(new_n250), .c(new_n245), .d(new_n251), .o1(\s[24] ));
  nano32aa1n03x7               g157(.a(new_n216), .b(new_n250), .c(new_n235), .d(new_n242), .out0(new_n253));
  oaih12aa1n02x5               g158(.a(new_n253), .b(new_n190), .c(new_n189), .o1(new_n254));
  aoai13aa1n06x5               g159(.a(new_n235), .b(new_n220), .c(new_n215), .d(new_n206), .o1(new_n255));
  and002aa1n12x5               g160(.a(new_n250), .b(new_n242), .o(new_n256));
  inv000aa1n04x5               g161(.a(new_n256), .o1(new_n257));
  oao003aa1n02x5               g162(.a(\a[24] ), .b(\b[23] ), .c(new_n247), .carry(new_n258));
  aoai13aa1n12x5               g163(.a(new_n258), .b(new_n257), .c(new_n255), .d(new_n238), .o1(new_n259));
  inv000aa1d42x5               g164(.a(new_n259), .o1(new_n260));
  xorc02aa1n12x5               g165(.a(\a[25] ), .b(\b[24] ), .out0(new_n261));
  xnbna2aa1n03x5               g166(.a(new_n261), .b(new_n254), .c(new_n260), .out0(\s[25] ));
  aoai13aa1n03x5               g167(.a(new_n261), .b(new_n259), .c(new_n185), .d(new_n253), .o1(new_n263));
  nor042aa1n03x5               g168(.a(\b[24] ), .b(\a[25] ), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n264), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n261), .o1(new_n266));
  aoai13aa1n02x5               g171(.a(new_n265), .b(new_n266), .c(new_n254), .d(new_n260), .o1(new_n267));
  xorc02aa1n02x5               g172(.a(\a[26] ), .b(\b[25] ), .out0(new_n268));
  norp02aa1n02x5               g173(.a(new_n268), .b(new_n264), .o1(new_n269));
  aoi022aa1n03x5               g174(.a(new_n267), .b(new_n268), .c(new_n263), .d(new_n269), .o1(\s[26] ));
  and002aa1n02x5               g175(.a(new_n268), .b(new_n261), .o(new_n271));
  inv000aa1n02x5               g176(.a(new_n271), .o1(new_n272));
  nano23aa1n06x5               g177(.a(new_n272), .b(new_n216), .c(new_n256), .d(new_n235), .out0(new_n273));
  oai012aa1n06x5               g178(.a(new_n273), .b(new_n190), .c(new_n189), .o1(new_n274));
  oao003aa1n02x5               g179(.a(\a[26] ), .b(\b[25] ), .c(new_n265), .carry(new_n275));
  aobi12aa1n06x5               g180(.a(new_n275), .b(new_n259), .c(new_n271), .out0(new_n276));
  xorc02aa1n12x5               g181(.a(\a[27] ), .b(\b[26] ), .out0(new_n277));
  xnbna2aa1n03x5               g182(.a(new_n277), .b(new_n276), .c(new_n274), .out0(\s[27] ));
  aoai13aa1n06x5               g183(.a(new_n256), .b(new_n239), .c(new_n221), .d(new_n235), .o1(new_n279));
  aoai13aa1n06x5               g184(.a(new_n275), .b(new_n272), .c(new_n279), .d(new_n258), .o1(new_n280));
  aoai13aa1n03x5               g185(.a(new_n277), .b(new_n280), .c(new_n185), .d(new_n273), .o1(new_n281));
  norp02aa1n02x5               g186(.a(\b[26] ), .b(\a[27] ), .o1(new_n282));
  inv000aa1n03x5               g187(.a(new_n282), .o1(new_n283));
  inv000aa1d42x5               g188(.a(new_n277), .o1(new_n284));
  aoai13aa1n03x5               g189(.a(new_n283), .b(new_n284), .c(new_n276), .d(new_n274), .o1(new_n285));
  xorc02aa1n02x5               g190(.a(\a[28] ), .b(\b[27] ), .out0(new_n286));
  norp02aa1n02x5               g191(.a(new_n286), .b(new_n282), .o1(new_n287));
  aoi022aa1n03x5               g192(.a(new_n285), .b(new_n286), .c(new_n281), .d(new_n287), .o1(\s[28] ));
  and002aa1n02x5               g193(.a(new_n286), .b(new_n277), .o(new_n289));
  aoai13aa1n03x5               g194(.a(new_n289), .b(new_n280), .c(new_n185), .d(new_n273), .o1(new_n290));
  inv000aa1d42x5               g195(.a(new_n289), .o1(new_n291));
  oao003aa1n02x5               g196(.a(\a[28] ), .b(\b[27] ), .c(new_n283), .carry(new_n292));
  aoai13aa1n03x5               g197(.a(new_n292), .b(new_n291), .c(new_n276), .d(new_n274), .o1(new_n293));
  xorc02aa1n02x5               g198(.a(\a[29] ), .b(\b[28] ), .out0(new_n294));
  norb02aa1n02x5               g199(.a(new_n292), .b(new_n294), .out0(new_n295));
  aoi022aa1n03x5               g200(.a(new_n293), .b(new_n294), .c(new_n290), .d(new_n295), .o1(\s[29] ));
  xnrb03aa1n02x5               g201(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n03x7               g202(.a(new_n284), .b(new_n286), .c(new_n294), .out0(new_n298));
  aoai13aa1n03x5               g203(.a(new_n298), .b(new_n280), .c(new_n185), .d(new_n273), .o1(new_n299));
  inv000aa1d42x5               g204(.a(new_n298), .o1(new_n300));
  oao003aa1n02x5               g205(.a(\a[29] ), .b(\b[28] ), .c(new_n292), .carry(new_n301));
  aoai13aa1n03x5               g206(.a(new_n301), .b(new_n300), .c(new_n276), .d(new_n274), .o1(new_n302));
  xorc02aa1n02x5               g207(.a(\a[30] ), .b(\b[29] ), .out0(new_n303));
  norb02aa1n02x5               g208(.a(new_n301), .b(new_n303), .out0(new_n304));
  aoi022aa1n03x5               g209(.a(new_n302), .b(new_n303), .c(new_n299), .d(new_n304), .o1(\s[30] ));
  xorc02aa1n02x5               g210(.a(\a[31] ), .b(\b[30] ), .out0(new_n306));
  nano32aa1n06x5               g211(.a(new_n284), .b(new_n303), .c(new_n286), .d(new_n294), .out0(new_n307));
  aoai13aa1n02x7               g212(.a(new_n307), .b(new_n280), .c(new_n185), .d(new_n273), .o1(new_n308));
  inv000aa1d42x5               g213(.a(new_n307), .o1(new_n309));
  oao003aa1n02x5               g214(.a(\a[30] ), .b(\b[29] ), .c(new_n301), .carry(new_n310));
  aoai13aa1n03x5               g215(.a(new_n310), .b(new_n309), .c(new_n276), .d(new_n274), .o1(new_n311));
  and002aa1n02x5               g216(.a(\b[29] ), .b(\a[30] ), .o(new_n312));
  oabi12aa1n02x5               g217(.a(new_n306), .b(\a[30] ), .c(\b[29] ), .out0(new_n313));
  oab012aa1n02x4               g218(.a(new_n313), .b(new_n301), .c(new_n312), .out0(new_n314));
  aoi022aa1n03x5               g219(.a(new_n311), .b(new_n306), .c(new_n308), .d(new_n314), .o1(\s[31] ));
  xorb03aa1n02x5               g220(.a(new_n100), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oai012aa1n02x5               g221(.a(new_n104), .b(new_n100), .c(new_n103), .o1(new_n317));
  xnrb03aa1n02x5               g222(.a(new_n317), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xnrc02aa1n02x5               g223(.a(new_n107), .b(new_n113), .out0(\s[5] ));
  norb02aa1n02x5               g224(.a(new_n109), .b(new_n108), .out0(new_n320));
  orn002aa1n02x5               g225(.a(\a[5] ), .b(\b[4] ), .o(new_n321));
  aoai13aa1n02x5               g226(.a(new_n113), .b(new_n106), .c(new_n105), .d(new_n100), .o1(new_n322));
  nanb03aa1n02x5               g227(.a(new_n117), .b(new_n322), .c(new_n109), .out0(new_n323));
  aoai13aa1n02x5               g228(.a(new_n323), .b(new_n320), .c(new_n322), .d(new_n321), .o1(\s[6] ));
  inv000aa1d42x5               g229(.a(new_n110), .o1(new_n325));
  aoi022aa1n02x5               g230(.a(new_n323), .b(new_n109), .c(new_n325), .d(new_n111), .o1(new_n326));
  nanp02aa1n02x5               g231(.a(new_n323), .b(new_n118), .o1(new_n327));
  norb02aa1n02x5               g232(.a(new_n327), .b(new_n326), .out0(\s[7] ));
  xnbna2aa1n03x5               g233(.a(new_n114), .b(new_n327), .c(new_n325), .out0(\s[8] ));
  xorb03aa1n02x5               g234(.a(new_n121), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


