// Benchmark "adder" written by ABC on Thu Jul 18 05:16:04 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n194, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n270, new_n271, new_n272, new_n273, new_n274, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n311, new_n313, new_n314, new_n315, new_n317, new_n318,
    new_n320, new_n321, new_n322, new_n324, new_n325, new_n326, new_n328,
    new_n329;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n03x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  and002aa1n03x5               g002(.a(\b[9] ), .b(\a[10] ), .o(new_n98));
  nor042aa1n02x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  nor042aa1n02x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  and002aa1n02x5               g005(.a(\b[8] ), .b(\a[9] ), .o(new_n101));
  orn002aa1n02x5               g006(.a(\a[2] ), .b(\b[1] ), .o(new_n102));
  nand22aa1n03x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  nand22aa1n04x5               g008(.a(\b[0] ), .b(\a[1] ), .o1(new_n104));
  aob012aa1n03x5               g009(.a(new_n102), .b(new_n103), .c(new_n104), .out0(new_n105));
  tech160nm_fixorc02aa1n05x5   g010(.a(\a[3] ), .b(\b[2] ), .out0(new_n106));
  oai022aa1n02x5               g011(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n107));
  tech160nm_fiaoi012aa1n05x5   g012(.a(new_n107), .b(new_n105), .c(new_n106), .o1(new_n108));
  nor002aa1n16x5               g013(.a(\b[6] ), .b(\a[7] ), .o1(new_n109));
  nand42aa1n04x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n09x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nano22aa1n03x7               g016(.a(new_n109), .b(new_n110), .c(new_n111), .out0(new_n112));
  nanp02aa1n02x5               g017(.a(\b[3] ), .b(\a[4] ), .o1(new_n113));
  nor042aa1n04x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nand02aa1n03x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  nano22aa1n03x7               g020(.a(new_n114), .b(new_n113), .c(new_n115), .out0(new_n116));
  nor042aa1n04x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nand02aa1d16x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  nor042aa1n04x5               g023(.a(\b[5] ), .b(\a[6] ), .o1(new_n119));
  norb03aa1n03x5               g024(.a(new_n118), .b(new_n117), .c(new_n119), .out0(new_n120));
  nand23aa1n03x5               g025(.a(new_n116), .b(new_n112), .c(new_n120), .o1(new_n121));
  nano23aa1n09x5               g026(.a(new_n117), .b(new_n109), .c(new_n110), .d(new_n111), .out0(new_n122));
  inv000aa1n02x5               g027(.a(new_n118), .o1(new_n123));
  oab012aa1n03x5               g028(.a(new_n123), .b(new_n114), .c(new_n119), .out0(new_n124));
  oaih12aa1n02x5               g029(.a(new_n110), .b(new_n117), .c(new_n109), .o1(new_n125));
  aobi12aa1n09x5               g030(.a(new_n125), .b(new_n122), .c(new_n124), .out0(new_n126));
  oai012aa1n12x5               g031(.a(new_n126), .b(new_n108), .c(new_n121), .o1(new_n127));
  aoib12aa1n02x5               g032(.a(new_n100), .b(new_n127), .c(new_n101), .out0(new_n128));
  xnrc02aa1n02x5               g033(.a(new_n128), .b(new_n99), .out0(\s[10] ));
  nand42aa1n03x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  nor002aa1d32x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  inv000aa1d42x5               g036(.a(new_n131), .o1(new_n132));
  norp02aa1n02x5               g037(.a(\b[1] ), .b(\a[2] ), .o1(new_n133));
  aoi012aa1n02x5               g038(.a(new_n133), .b(new_n103), .c(new_n104), .o1(new_n134));
  xnrc02aa1n02x5               g039(.a(\b[2] ), .b(\a[3] ), .out0(new_n135));
  oabi12aa1n03x5               g040(.a(new_n107), .b(new_n135), .c(new_n134), .out0(new_n136));
  nanb03aa1n02x5               g041(.a(new_n114), .b(new_n115), .c(new_n113), .out0(new_n137));
  nano22aa1n02x4               g042(.a(new_n137), .b(new_n112), .c(new_n120), .out0(new_n138));
  nanp02aa1n03x5               g043(.a(new_n138), .b(new_n136), .o1(new_n139));
  norp02aa1n02x5               g044(.a(new_n100), .b(new_n97), .o1(new_n140));
  aoai13aa1n02x5               g045(.a(new_n140), .b(new_n101), .c(new_n139), .d(new_n126), .o1(new_n141));
  aboi22aa1n03x5               g046(.a(new_n98), .b(new_n141), .c(new_n130), .d(new_n132), .out0(new_n142));
  aoi022aa1d24x5               g047(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n143));
  nanp03aa1n02x5               g048(.a(new_n141), .b(new_n132), .c(new_n143), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n144), .b(new_n142), .out0(\s[11] ));
  nor022aa1n04x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nand42aa1d28x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  norb02aa1n02x5               g052(.a(new_n147), .b(new_n146), .out0(new_n148));
  nor002aa1n02x5               g053(.a(new_n146), .b(new_n131), .o1(new_n149));
  nanp03aa1n02x5               g054(.a(new_n144), .b(new_n147), .c(new_n149), .o1(new_n150));
  aoai13aa1n02x5               g055(.a(new_n150), .b(new_n148), .c(new_n132), .d(new_n144), .o1(\s[12] ));
  nano23aa1n03x5               g056(.a(new_n146), .b(new_n131), .c(new_n147), .d(new_n130), .out0(new_n152));
  norp02aa1n02x5               g057(.a(new_n101), .b(new_n100), .o1(new_n153));
  nand23aa1n03x5               g058(.a(new_n152), .b(new_n99), .c(new_n153), .o1(new_n154));
  oai012aa1n03x5               g059(.a(new_n143), .b(new_n100), .c(new_n97), .o1(new_n155));
  nand02aa1n04x5               g060(.a(new_n155), .b(new_n149), .o1(new_n156));
  nanp02aa1n02x5               g061(.a(new_n156), .b(new_n147), .o1(new_n157));
  aoai13aa1n06x5               g062(.a(new_n157), .b(new_n154), .c(new_n139), .d(new_n126), .o1(new_n158));
  xorb03aa1n02x5               g063(.a(new_n158), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1d32x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  inv040aa1n08x5               g065(.a(new_n160), .o1(new_n161));
  nand02aa1n06x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  nanp03aa1n02x5               g067(.a(new_n158), .b(new_n161), .c(new_n162), .o1(new_n163));
  nor002aa1n03x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  tech160nm_finand02aa1n05x5   g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  norb02aa1n02x5               g070(.a(new_n165), .b(new_n164), .out0(new_n166));
  nona23aa1n02x4               g071(.a(new_n163), .b(new_n165), .c(new_n164), .d(new_n160), .out0(new_n167));
  aoai13aa1n02x5               g072(.a(new_n167), .b(new_n166), .c(new_n163), .d(new_n161), .o1(\s[14] ));
  nano23aa1n06x5               g073(.a(new_n160), .b(new_n164), .c(new_n165), .d(new_n162), .out0(new_n169));
  oaoi03aa1n06x5               g074(.a(\a[14] ), .b(\b[13] ), .c(new_n161), .o1(new_n170));
  nor002aa1n20x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  tech160nm_finand02aa1n03p5x5 g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  aoai13aa1n03x5               g078(.a(new_n173), .b(new_n170), .c(new_n158), .d(new_n169), .o1(new_n174));
  aoi112aa1n02x5               g079(.a(new_n173), .b(new_n170), .c(new_n158), .d(new_n169), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n174), .b(new_n175), .out0(\s[15] ));
  inv000aa1d42x5               g081(.a(new_n171), .o1(new_n177));
  nor002aa1n03x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nand42aa1n03x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n179), .b(new_n178), .out0(new_n180));
  aobi12aa1n02x5               g085(.a(new_n180), .b(new_n174), .c(new_n177), .out0(new_n181));
  nona22aa1n02x4               g086(.a(new_n174), .b(new_n180), .c(new_n171), .out0(new_n182));
  norb02aa1n03x4               g087(.a(new_n182), .b(new_n181), .out0(\s[16] ));
  inv040aa1d32x5               g088(.a(\a[17] ), .o1(new_n184));
  nano23aa1n06x5               g089(.a(new_n171), .b(new_n178), .c(new_n179), .d(new_n172), .out0(new_n185));
  nano22aa1n09x5               g090(.a(new_n154), .b(new_n169), .c(new_n185), .out0(new_n186));
  inv000aa1d42x5               g091(.a(new_n147), .o1(new_n187));
  aoi022aa1n02x5               g092(.a(\b[15] ), .b(\a[16] ), .c(\a[15] ), .d(\b[14] ), .o1(new_n188));
  oaoi13aa1n06x5               g093(.a(new_n178), .b(new_n188), .c(new_n170), .d(new_n171), .o1(new_n189));
  nand03aa1n04x5               g094(.a(new_n156), .b(new_n169), .c(new_n185), .o1(new_n190));
  oai012aa1n12x5               g095(.a(new_n189), .b(new_n190), .c(new_n187), .o1(new_n191));
  aoi012aa1n06x5               g096(.a(new_n191), .b(new_n127), .c(new_n186), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[16] ), .c(new_n184), .out0(\s[17] ));
  oaoi03aa1n03x5               g098(.a(\a[17] ), .b(\b[16] ), .c(new_n192), .o1(new_n194));
  xorb03aa1n02x5               g099(.a(new_n194), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  inv000aa1d42x5               g100(.a(\a[18] ), .o1(new_n196));
  xroi22aa1d06x4               g101(.a(new_n184), .b(\b[16] ), .c(new_n196), .d(\b[17] ), .out0(new_n197));
  aoai13aa1n06x5               g102(.a(new_n197), .b(new_n191), .c(new_n127), .d(new_n186), .o1(new_n198));
  nanb02aa1n12x5               g103(.a(\b[16] ), .b(new_n184), .out0(new_n199));
  oaoi03aa1n12x5               g104(.a(\a[18] ), .b(\b[17] ), .c(new_n199), .o1(new_n200));
  inv000aa1d42x5               g105(.a(new_n200), .o1(new_n201));
  nor042aa1n09x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  nand02aa1n03x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  nanb02aa1n02x5               g108(.a(new_n202), .b(new_n203), .out0(new_n204));
  xobna2aa1n03x5               g109(.a(new_n204), .b(new_n198), .c(new_n201), .out0(\s[19] ));
  xnrc02aa1n02x5               g110(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g111(.a(new_n202), .o1(new_n207));
  ao0012aa1n03x7               g112(.a(new_n204), .b(new_n198), .c(new_n201), .o(new_n208));
  nor042aa1n02x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  nand02aa1n16x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  norb02aa1n02x5               g115(.a(new_n210), .b(new_n209), .out0(new_n211));
  inv000aa1d42x5               g116(.a(new_n210), .o1(new_n212));
  oai022aa1n02x5               g117(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n213));
  norp02aa1n02x5               g118(.a(new_n213), .b(new_n212), .o1(new_n214));
  aoai13aa1n02x5               g119(.a(new_n214), .b(new_n204), .c(new_n198), .d(new_n201), .o1(new_n215));
  aoai13aa1n02x5               g120(.a(new_n215), .b(new_n211), .c(new_n208), .d(new_n207), .o1(\s[20] ));
  nano23aa1n09x5               g121(.a(new_n202), .b(new_n209), .c(new_n210), .d(new_n203), .out0(new_n217));
  nand02aa1d04x5               g122(.a(new_n197), .b(new_n217), .o1(new_n218));
  inv000aa1d42x5               g123(.a(new_n218), .o1(new_n219));
  aoai13aa1n04x5               g124(.a(new_n219), .b(new_n191), .c(new_n127), .d(new_n186), .o1(new_n220));
  aoi022aa1n02x5               g125(.a(new_n217), .b(new_n200), .c(new_n210), .d(new_n213), .o1(new_n221));
  tech160nm_fixnrc02aa1n05x5   g126(.a(\b[20] ), .b(\a[21] ), .out0(new_n222));
  xobna2aa1n03x5               g127(.a(new_n222), .b(new_n220), .c(new_n221), .out0(\s[21] ));
  tech160nm_fiaoi012aa1n05x5   g128(.a(new_n222), .b(new_n220), .c(new_n221), .o1(new_n224));
  xnrc02aa1n02x5               g129(.a(\b[21] ), .b(\a[22] ), .out0(new_n225));
  norp02aa1n02x5               g130(.a(\b[20] ), .b(\a[21] ), .o1(new_n226));
  norp02aa1n02x5               g131(.a(new_n224), .b(new_n226), .o1(new_n227));
  nanp02aa1n02x5               g132(.a(\b[21] ), .b(\a[22] ), .o1(new_n228));
  oai022aa1n02x5               g133(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n229));
  nanb02aa1n02x5               g134(.a(new_n229), .b(new_n228), .out0(new_n230));
  obai22aa1n02x7               g135(.a(new_n225), .b(new_n227), .c(new_n224), .d(new_n230), .out0(\s[22] ));
  nor042aa1n02x5               g136(.a(new_n225), .b(new_n222), .o1(new_n232));
  and003aa1n02x5               g137(.a(new_n197), .b(new_n232), .c(new_n217), .o(new_n233));
  aoai13aa1n06x5               g138(.a(new_n233), .b(new_n191), .c(new_n127), .d(new_n186), .o1(new_n234));
  oab012aa1n02x4               g139(.a(new_n212), .b(new_n202), .c(new_n209), .out0(new_n235));
  aoai13aa1n06x5               g140(.a(new_n232), .b(new_n235), .c(new_n217), .d(new_n200), .o1(new_n236));
  nanp02aa1n02x5               g141(.a(new_n229), .b(new_n228), .o1(new_n237));
  nanp02aa1n02x5               g142(.a(new_n236), .b(new_n237), .o1(new_n238));
  inv000aa1n02x5               g143(.a(new_n238), .o1(new_n239));
  nor042aa1n04x5               g144(.a(\b[22] ), .b(\a[23] ), .o1(new_n240));
  nand42aa1n02x5               g145(.a(\b[22] ), .b(\a[23] ), .o1(new_n241));
  nanb02aa1n02x5               g146(.a(new_n240), .b(new_n241), .out0(new_n242));
  xobna2aa1n03x5               g147(.a(new_n242), .b(new_n234), .c(new_n239), .out0(\s[23] ));
  inv000aa1d42x5               g148(.a(new_n240), .o1(new_n244));
  ao0012aa1n03x7               g149(.a(new_n242), .b(new_n234), .c(new_n239), .o(new_n245));
  nor042aa1n02x5               g150(.a(\b[23] ), .b(\a[24] ), .o1(new_n246));
  nand42aa1n03x5               g151(.a(\b[23] ), .b(\a[24] ), .o1(new_n247));
  norb02aa1n02x5               g152(.a(new_n247), .b(new_n246), .out0(new_n248));
  norb03aa1n02x5               g153(.a(new_n247), .b(new_n240), .c(new_n246), .out0(new_n249));
  aoai13aa1n02x5               g154(.a(new_n249), .b(new_n242), .c(new_n234), .d(new_n239), .o1(new_n250));
  aoai13aa1n02x5               g155(.a(new_n250), .b(new_n248), .c(new_n245), .d(new_n244), .o1(\s[24] ));
  nano23aa1n03x7               g156(.a(new_n240), .b(new_n246), .c(new_n247), .d(new_n241), .out0(new_n252));
  nano22aa1n02x4               g157(.a(new_n218), .b(new_n232), .c(new_n252), .out0(new_n253));
  aoai13aa1n06x5               g158(.a(new_n253), .b(new_n191), .c(new_n127), .d(new_n186), .o1(new_n254));
  inv040aa1n03x5               g159(.a(new_n252), .o1(new_n255));
  oai012aa1n02x5               g160(.a(new_n247), .b(new_n246), .c(new_n240), .o1(new_n256));
  aoai13aa1n06x5               g161(.a(new_n256), .b(new_n255), .c(new_n236), .d(new_n237), .o1(new_n257));
  inv020aa1n02x5               g162(.a(new_n257), .o1(new_n258));
  tech160nm_fixorc02aa1n03p5x5 g163(.a(\a[25] ), .b(\b[24] ), .out0(new_n259));
  xnbna2aa1n03x5               g164(.a(new_n259), .b(new_n254), .c(new_n258), .out0(\s[25] ));
  norp02aa1n02x5               g165(.a(\b[24] ), .b(\a[25] ), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n261), .o1(new_n262));
  aob012aa1n03x5               g167(.a(new_n259), .b(new_n254), .c(new_n258), .out0(new_n263));
  xorc02aa1n02x5               g168(.a(\a[26] ), .b(\b[25] ), .out0(new_n264));
  nanp02aa1n02x5               g169(.a(\b[25] ), .b(\a[26] ), .o1(new_n265));
  oai022aa1n02x5               g170(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n266));
  norb02aa1n02x5               g171(.a(new_n265), .b(new_n266), .out0(new_n267));
  nand42aa1n02x5               g172(.a(new_n263), .b(new_n267), .o1(new_n268));
  aoai13aa1n03x5               g173(.a(new_n268), .b(new_n264), .c(new_n262), .d(new_n263), .o1(\s[26] ));
  and002aa1n02x5               g174(.a(new_n264), .b(new_n259), .o(new_n270));
  nano32aa1n03x7               g175(.a(new_n218), .b(new_n270), .c(new_n232), .d(new_n252), .out0(new_n271));
  aoai13aa1n12x5               g176(.a(new_n271), .b(new_n191), .c(new_n127), .d(new_n186), .o1(new_n272));
  aoi022aa1n06x5               g177(.a(new_n257), .b(new_n270), .c(new_n265), .d(new_n266), .o1(new_n273));
  xorc02aa1n12x5               g178(.a(\a[27] ), .b(\b[26] ), .out0(new_n274));
  xnbna2aa1n03x5               g179(.a(new_n274), .b(new_n273), .c(new_n272), .out0(\s[27] ));
  nanp02aa1n02x5               g180(.a(new_n257), .b(new_n270), .o1(new_n276));
  nanp02aa1n02x5               g181(.a(new_n266), .b(new_n265), .o1(new_n277));
  nanp03aa1n06x5               g182(.a(new_n276), .b(new_n272), .c(new_n277), .o1(new_n278));
  norp02aa1n02x5               g183(.a(\b[26] ), .b(\a[27] ), .o1(new_n279));
  xnrc02aa1n12x5               g184(.a(\b[27] ), .b(\a[28] ), .out0(new_n280));
  aoai13aa1n03x5               g185(.a(new_n280), .b(new_n279), .c(new_n278), .d(new_n274), .o1(new_n281));
  inv000aa1d42x5               g186(.a(new_n274), .o1(new_n282));
  norp02aa1n02x5               g187(.a(new_n280), .b(new_n279), .o1(new_n283));
  aoai13aa1n02x7               g188(.a(new_n283), .b(new_n282), .c(new_n273), .d(new_n272), .o1(new_n284));
  nanp02aa1n03x5               g189(.a(new_n281), .b(new_n284), .o1(\s[28] ));
  norb02aa1n03x5               g190(.a(new_n274), .b(new_n280), .out0(new_n286));
  nanp02aa1n03x5               g191(.a(new_n278), .b(new_n286), .o1(new_n287));
  inv000aa1d42x5               g192(.a(new_n286), .o1(new_n288));
  nanp02aa1n02x5               g193(.a(\b[27] ), .b(\a[28] ), .o1(new_n289));
  oai012aa1n02x5               g194(.a(new_n289), .b(new_n280), .c(new_n279), .o1(new_n290));
  aoai13aa1n02x7               g195(.a(new_n290), .b(new_n288), .c(new_n273), .d(new_n272), .o1(new_n291));
  xorc02aa1n02x5               g196(.a(\a[29] ), .b(\b[28] ), .out0(new_n292));
  oaoi13aa1n02x5               g197(.a(new_n292), .b(new_n289), .c(new_n280), .d(new_n279), .o1(new_n293));
  aoi022aa1n03x5               g198(.a(new_n291), .b(new_n292), .c(new_n287), .d(new_n293), .o1(\s[29] ));
  xorb03aa1n02x5               g199(.a(new_n104), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g200(.a(new_n280), .b(new_n274), .c(new_n292), .out0(new_n296));
  nanp02aa1n03x5               g201(.a(new_n278), .b(new_n296), .o1(new_n297));
  inv000aa1n02x5               g202(.a(new_n296), .o1(new_n298));
  oao003aa1n02x5               g203(.a(\a[29] ), .b(\b[28] ), .c(new_n290), .carry(new_n299));
  aoai13aa1n02x7               g204(.a(new_n299), .b(new_n298), .c(new_n273), .d(new_n272), .o1(new_n300));
  xorc02aa1n02x5               g205(.a(\a[30] ), .b(\b[29] ), .out0(new_n301));
  norb02aa1n02x5               g206(.a(new_n299), .b(new_n301), .out0(new_n302));
  aoi022aa1n03x5               g207(.a(new_n300), .b(new_n301), .c(new_n297), .d(new_n302), .o1(\s[30] ));
  nand03aa1n02x5               g208(.a(new_n286), .b(new_n292), .c(new_n301), .o1(new_n304));
  nanb02aa1n03x5               g209(.a(new_n304), .b(new_n278), .out0(new_n305));
  oao003aa1n02x5               g210(.a(\a[30] ), .b(\b[29] ), .c(new_n299), .carry(new_n306));
  aoai13aa1n02x7               g211(.a(new_n306), .b(new_n304), .c(new_n273), .d(new_n272), .o1(new_n307));
  xorc02aa1n02x5               g212(.a(\a[31] ), .b(\b[30] ), .out0(new_n308));
  norb02aa1n02x5               g213(.a(new_n306), .b(new_n308), .out0(new_n309));
  aoi022aa1n03x5               g214(.a(new_n307), .b(new_n308), .c(new_n305), .d(new_n309), .o1(\s[31] ));
  inv000aa1d42x5               g215(.a(\a[3] ), .o1(new_n311));
  xorb03aa1n02x5               g216(.a(new_n134), .b(\b[2] ), .c(new_n311), .out0(\s[3] ));
  nanp02aa1n02x5               g217(.a(new_n105), .b(new_n106), .o1(new_n313));
  xorc02aa1n02x5               g218(.a(\a[4] ), .b(\b[3] ), .out0(new_n314));
  aoib12aa1n02x5               g219(.a(new_n314), .b(new_n311), .c(\b[2] ), .out0(new_n315));
  aoi022aa1n02x5               g220(.a(new_n136), .b(new_n314), .c(new_n315), .d(new_n313), .o1(\s[4] ));
  aoai13aa1n02x5               g221(.a(new_n116), .b(new_n107), .c(new_n105), .d(new_n106), .o1(new_n317));
  aboi22aa1n03x5               g222(.a(new_n114), .b(new_n115), .c(new_n136), .d(new_n113), .out0(new_n318));
  norb02aa1n02x5               g223(.a(new_n317), .b(new_n318), .out0(\s[5] ));
  norb02aa1n02x5               g224(.a(new_n118), .b(new_n119), .out0(new_n320));
  aoi012aa1n02x5               g225(.a(new_n114), .b(new_n136), .c(new_n116), .o1(new_n321));
  nona32aa1n02x4               g226(.a(new_n317), .b(new_n119), .c(new_n123), .d(new_n114), .out0(new_n322));
  oai012aa1n02x5               g227(.a(new_n322), .b(new_n321), .c(new_n320), .o1(\s[6] ));
  inv000aa1d42x5               g228(.a(new_n109), .o1(new_n324));
  aoi022aa1n02x5               g229(.a(new_n322), .b(new_n118), .c(new_n324), .d(new_n111), .o1(new_n325));
  nano32aa1n02x4               g230(.a(new_n109), .b(new_n322), .c(new_n111), .d(new_n118), .out0(new_n326));
  norp02aa1n02x5               g231(.a(new_n326), .b(new_n325), .o1(\s[7] ));
  norb02aa1n02x5               g232(.a(new_n110), .b(new_n117), .out0(new_n328));
  nona23aa1n02x4               g233(.a(new_n322), .b(new_n111), .c(new_n109), .d(new_n123), .out0(new_n329));
  xnbna2aa1n03x5               g234(.a(new_n328), .b(new_n329), .c(new_n324), .out0(\s[8] ));
  xorb03aa1n02x5               g235(.a(new_n127), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


