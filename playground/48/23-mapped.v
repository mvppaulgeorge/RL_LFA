// Benchmark "adder" written by ABC on Thu Jul 18 12:47:54 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n189, new_n190, new_n191, new_n192, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n320, new_n323, new_n324, new_n326, new_n327,
    new_n329;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  tech160nm_fixnrc02aa1n04x5   g001(.a(\b[9] ), .b(\a[10] ), .out0(new_n97));
  inv000aa1n02x5               g002(.a(new_n97), .o1(new_n98));
  nor002aa1n02x5               g003(.a(\b[8] ), .b(\a[9] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(new_n99), .o1(new_n100));
  and002aa1n02x5               g005(.a(\b[8] ), .b(\a[9] ), .o(new_n101));
  inv000aa1d42x5               g006(.a(new_n101), .o1(new_n102));
  nor002aa1n06x5               g007(.a(\b[7] ), .b(\a[8] ), .o1(new_n103));
  nanp02aa1n04x5               g008(.a(\b[7] ), .b(\a[8] ), .o1(new_n104));
  nor022aa1n16x5               g009(.a(\b[6] ), .b(\a[7] ), .o1(new_n105));
  nand02aa1n03x5               g010(.a(\b[6] ), .b(\a[7] ), .o1(new_n106));
  nona23aa1n09x5               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  tech160nm_fixnrc02aa1n05x5   g012(.a(\b[5] ), .b(\a[6] ), .out0(new_n108));
  tech160nm_fixnrc02aa1n02p5x5 g013(.a(\b[4] ), .b(\a[5] ), .out0(new_n109));
  nor043aa1n03x5               g014(.a(new_n107), .b(new_n108), .c(new_n109), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[1] ), .b(\a[2] ), .o1(new_n111));
  nand22aa1n03x5               g016(.a(\b[0] ), .b(\a[1] ), .o1(new_n112));
  nor042aa1n02x5               g017(.a(\b[1] ), .b(\a[2] ), .o1(new_n113));
  oai012aa1n06x5               g018(.a(new_n111), .b(new_n113), .c(new_n112), .o1(new_n114));
  nor022aa1n06x5               g019(.a(\b[3] ), .b(\a[4] ), .o1(new_n115));
  nanp02aa1n04x5               g020(.a(\b[3] ), .b(\a[4] ), .o1(new_n116));
  nor022aa1n16x5               g021(.a(\b[2] ), .b(\a[3] ), .o1(new_n117));
  nand42aa1n02x5               g022(.a(\b[2] ), .b(\a[3] ), .o1(new_n118));
  nona23aa1n09x5               g023(.a(new_n118), .b(new_n116), .c(new_n115), .d(new_n117), .out0(new_n119));
  tech160nm_fioai012aa1n03p5x5 g024(.a(new_n116), .b(new_n117), .c(new_n115), .o1(new_n120));
  oai012aa1n12x5               g025(.a(new_n120), .b(new_n119), .c(new_n114), .o1(new_n121));
  nand42aa1n03x5               g026(.a(new_n105), .b(new_n104), .o1(new_n122));
  inv040aa1d32x5               g027(.a(\a[6] ), .o1(new_n123));
  oai022aa1n04x7               g028(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n124));
  oaib12aa1n06x5               g029(.a(new_n124), .b(new_n123), .c(\b[5] ), .out0(new_n125));
  oai122aa1n12x5               g030(.a(new_n122), .b(new_n107), .c(new_n125), .d(\b[7] ), .e(\a[8] ), .o1(new_n126));
  aoai13aa1n06x5               g031(.a(new_n102), .b(new_n126), .c(new_n110), .d(new_n121), .o1(new_n127));
  xnbna2aa1n03x5               g032(.a(new_n98), .b(new_n127), .c(new_n100), .out0(\s[10] ));
  tech160nm_fiaoi012aa1n05x5   g033(.a(new_n97), .b(new_n127), .c(new_n100), .o1(new_n129));
  nor002aa1d32x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  nand02aa1n08x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  norb02aa1n06x5               g036(.a(new_n131), .b(new_n130), .out0(new_n132));
  inv000aa1d42x5               g037(.a(\b[9] ), .o1(new_n133));
  oai022aa1d24x5               g038(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n134));
  oaib12aa1n18x5               g039(.a(new_n134), .b(new_n133), .c(\a[10] ), .out0(new_n135));
  inv040aa1n09x5               g040(.a(new_n135), .o1(new_n136));
  tech160nm_fioai012aa1n04x5   g041(.a(new_n132), .b(new_n129), .c(new_n136), .o1(new_n137));
  inv000aa1d42x5               g042(.a(new_n132), .o1(new_n138));
  nano22aa1n02x4               g043(.a(new_n129), .b(new_n138), .c(new_n135), .out0(new_n139));
  norb02aa1n02x5               g044(.a(new_n137), .b(new_n139), .out0(\s[11] ));
  nor042aa1n06x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand02aa1n06x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  nona22aa1n06x5               g048(.a(new_n137), .b(new_n143), .c(new_n130), .out0(new_n144));
  inv040aa1n04x5               g049(.a(new_n130), .o1(new_n145));
  inv000aa1d42x5               g050(.a(new_n143), .o1(new_n146));
  tech160nm_fiaoi012aa1n03p5x5 g051(.a(new_n146), .b(new_n137), .c(new_n145), .o1(new_n147));
  norb02aa1n03x4               g052(.a(new_n144), .b(new_n147), .out0(\s[12] ));
  nona23aa1n12x5               g053(.a(new_n142), .b(new_n131), .c(new_n130), .d(new_n141), .out0(new_n149));
  oaoi03aa1n09x5               g054(.a(\a[12] ), .b(\b[11] ), .c(new_n145), .o1(new_n150));
  oabi12aa1n18x5               g055(.a(new_n150), .b(new_n149), .c(new_n135), .out0(new_n151));
  aoi012aa1n09x5               g056(.a(new_n126), .b(new_n110), .c(new_n121), .o1(new_n152));
  xnrc02aa1n12x5               g057(.a(\b[8] ), .b(\a[9] ), .out0(new_n153));
  inv000aa1d42x5               g058(.a(new_n153), .o1(new_n154));
  norp02aa1n02x5               g059(.a(new_n149), .b(new_n97), .o1(new_n155));
  nano22aa1n03x7               g060(.a(new_n152), .b(new_n154), .c(new_n155), .out0(new_n156));
  norp02aa1n02x5               g061(.a(new_n156), .b(new_n151), .o1(new_n157));
  xnrb03aa1n02x5               g062(.a(new_n157), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n04x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  tech160nm_finand02aa1n03p5x5 g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  oaoi13aa1n03x5               g065(.a(new_n159), .b(new_n160), .c(new_n156), .d(new_n151), .o1(new_n161));
  xnrb03aa1n03x5               g066(.a(new_n161), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n02x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  tech160nm_finand02aa1n03p5x5 g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nano23aa1d12x5               g069(.a(new_n159), .b(new_n163), .c(new_n164), .d(new_n160), .out0(new_n165));
  nano32aa1n03x7               g070(.a(new_n152), .b(new_n165), .c(new_n154), .d(new_n155), .out0(new_n166));
  aoi012aa1n03x5               g071(.a(new_n163), .b(new_n159), .c(new_n164), .o1(new_n167));
  aobi12aa1n09x5               g072(.a(new_n167), .b(new_n151), .c(new_n165), .out0(new_n168));
  nor042aa1n09x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  nand02aa1n03x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(new_n171));
  oaib12aa1n06x5               g076(.a(new_n171), .b(new_n166), .c(new_n168), .out0(new_n172));
  norb03aa1n02x5               g077(.a(new_n168), .b(new_n166), .c(new_n171), .out0(new_n173));
  norb02aa1n02x5               g078(.a(new_n172), .b(new_n173), .out0(\s[15] ));
  inv000aa1d42x5               g079(.a(new_n169), .o1(new_n175));
  nor042aa1n02x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  nand02aa1n03x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  norb02aa1n02x5               g082(.a(new_n177), .b(new_n176), .out0(new_n178));
  xnbna2aa1n03x5               g083(.a(new_n178), .b(new_n172), .c(new_n175), .out0(\s[16] ));
  nano23aa1d15x5               g084(.a(new_n169), .b(new_n176), .c(new_n177), .d(new_n170), .out0(new_n180));
  inv000aa1n02x5               g085(.a(new_n180), .o1(new_n181));
  nano23aa1n06x5               g086(.a(new_n130), .b(new_n141), .c(new_n142), .d(new_n131), .out0(new_n182));
  nand42aa1n04x5               g087(.a(new_n180), .b(new_n165), .o1(new_n183));
  nano32aa1n03x7               g088(.a(new_n183), .b(new_n154), .c(new_n98), .d(new_n182), .out0(new_n184));
  aoai13aa1n04x5               g089(.a(new_n184), .b(new_n126), .c(new_n110), .d(new_n121), .o1(new_n185));
  tech160nm_fiaoi012aa1n03p5x5 g090(.a(new_n176), .b(new_n169), .c(new_n177), .o1(new_n186));
  oai112aa1n06x5               g091(.a(new_n185), .b(new_n186), .c(new_n168), .d(new_n181), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g093(.a(\a[18] ), .o1(new_n189));
  inv040aa1d32x5               g094(.a(\a[17] ), .o1(new_n190));
  inv000aa1d42x5               g095(.a(\b[16] ), .o1(new_n191));
  oaoi03aa1n03x5               g096(.a(new_n190), .b(new_n191), .c(new_n187), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[17] ), .c(new_n189), .out0(\s[18] ));
  inv030aa1n02x5               g098(.a(new_n184), .o1(new_n194));
  nor042aa1n09x5               g099(.a(new_n152), .b(new_n194), .o1(new_n195));
  aoai13aa1n06x5               g100(.a(new_n165), .b(new_n150), .c(new_n182), .d(new_n136), .o1(new_n196));
  aoai13aa1n06x5               g101(.a(new_n186), .b(new_n181), .c(new_n196), .d(new_n167), .o1(new_n197));
  xroi22aa1d06x4               g102(.a(new_n190), .b(\b[16] ), .c(new_n189), .d(\b[17] ), .out0(new_n198));
  tech160nm_fioai012aa1n05x5   g103(.a(new_n198), .b(new_n195), .c(new_n197), .o1(new_n199));
  oaih22aa1n06x5               g104(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n200));
  oaib12aa1n18x5               g105(.a(new_n200), .b(new_n189), .c(\b[17] ), .out0(new_n201));
  nor002aa1d32x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  nand42aa1n20x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  nanb02aa1n02x5               g108(.a(new_n202), .b(new_n203), .out0(new_n204));
  inv000aa1d42x5               g109(.a(new_n204), .o1(new_n205));
  xnbna2aa1n03x5               g110(.a(new_n205), .b(new_n199), .c(new_n201), .out0(\s[19] ));
  xnrc02aa1n02x5               g111(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g112(.a(new_n202), .o1(new_n208));
  aoi012aa1n03x5               g113(.a(new_n204), .b(new_n199), .c(new_n201), .o1(new_n209));
  nor002aa1d24x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  nand42aa1d28x5               g115(.a(\b[19] ), .b(\a[20] ), .o1(new_n211));
  nanb02aa1n02x5               g116(.a(new_n210), .b(new_n211), .out0(new_n212));
  nano22aa1n03x5               g117(.a(new_n209), .b(new_n208), .c(new_n212), .out0(new_n213));
  nanp02aa1n02x5               g118(.a(new_n191), .b(new_n190), .o1(new_n214));
  oaoi03aa1n03x5               g119(.a(\a[18] ), .b(\b[17] ), .c(new_n214), .o1(new_n215));
  aoai13aa1n02x7               g120(.a(new_n205), .b(new_n215), .c(new_n187), .d(new_n198), .o1(new_n216));
  aoi012aa1n03x5               g121(.a(new_n212), .b(new_n216), .c(new_n208), .o1(new_n217));
  nor002aa1n02x5               g122(.a(new_n217), .b(new_n213), .o1(\s[20] ));
  nano23aa1n09x5               g123(.a(new_n202), .b(new_n210), .c(new_n211), .d(new_n203), .out0(new_n219));
  nand02aa1d04x5               g124(.a(new_n198), .b(new_n219), .o1(new_n220));
  inv000aa1d42x5               g125(.a(new_n220), .o1(new_n221));
  tech160nm_fioai012aa1n05x5   g126(.a(new_n221), .b(new_n195), .c(new_n197), .o1(new_n222));
  nona23aa1n09x5               g127(.a(new_n211), .b(new_n203), .c(new_n202), .d(new_n210), .out0(new_n223));
  aoi012aa1d18x5               g128(.a(new_n210), .b(new_n202), .c(new_n211), .o1(new_n224));
  oai012aa1d24x5               g129(.a(new_n224), .b(new_n223), .c(new_n201), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  nor002aa1d32x5               g131(.a(\b[20] ), .b(\a[21] ), .o1(new_n227));
  nand42aa1n02x5               g132(.a(\b[20] ), .b(\a[21] ), .o1(new_n228));
  norb02aa1n02x5               g133(.a(new_n228), .b(new_n227), .out0(new_n229));
  xnbna2aa1n03x5               g134(.a(new_n229), .b(new_n222), .c(new_n226), .out0(\s[21] ));
  inv000aa1d42x5               g135(.a(new_n227), .o1(new_n231));
  aobi12aa1n03x5               g136(.a(new_n229), .b(new_n222), .c(new_n226), .out0(new_n232));
  xnrc02aa1n12x5               g137(.a(\b[21] ), .b(\a[22] ), .out0(new_n233));
  nano22aa1n02x4               g138(.a(new_n232), .b(new_n231), .c(new_n233), .out0(new_n234));
  aoai13aa1n03x5               g139(.a(new_n229), .b(new_n225), .c(new_n187), .d(new_n221), .o1(new_n235));
  aoi012aa1n03x5               g140(.a(new_n233), .b(new_n235), .c(new_n231), .o1(new_n236));
  nor002aa1n02x5               g141(.a(new_n236), .b(new_n234), .o1(\s[22] ));
  nano22aa1n09x5               g142(.a(new_n233), .b(new_n231), .c(new_n228), .out0(new_n238));
  and003aa1n02x5               g143(.a(new_n198), .b(new_n238), .c(new_n219), .o(new_n239));
  tech160nm_fioai012aa1n05x5   g144(.a(new_n239), .b(new_n195), .c(new_n197), .o1(new_n240));
  oao003aa1n12x5               g145(.a(\a[22] ), .b(\b[21] ), .c(new_n231), .carry(new_n241));
  inv000aa1d42x5               g146(.a(new_n241), .o1(new_n242));
  tech160nm_fiaoi012aa1n04x5   g147(.a(new_n242), .b(new_n225), .c(new_n238), .o1(new_n243));
  xnrc02aa1n12x5               g148(.a(\b[22] ), .b(\a[23] ), .out0(new_n244));
  inv000aa1d42x5               g149(.a(new_n244), .o1(new_n245));
  xnbna2aa1n03x5               g150(.a(new_n245), .b(new_n240), .c(new_n243), .out0(\s[23] ));
  nor042aa1n06x5               g151(.a(\b[22] ), .b(\a[23] ), .o1(new_n247));
  inv000aa1d42x5               g152(.a(new_n247), .o1(new_n248));
  aoi012aa1n03x5               g153(.a(new_n244), .b(new_n240), .c(new_n243), .o1(new_n249));
  tech160nm_fixnrc02aa1n02p5x5 g154(.a(\b[23] ), .b(\a[24] ), .out0(new_n250));
  nano22aa1n03x5               g155(.a(new_n249), .b(new_n248), .c(new_n250), .out0(new_n251));
  inv000aa1n02x5               g156(.a(new_n243), .o1(new_n252));
  aoai13aa1n04x5               g157(.a(new_n245), .b(new_n252), .c(new_n187), .d(new_n239), .o1(new_n253));
  aoi012aa1n03x5               g158(.a(new_n250), .b(new_n253), .c(new_n248), .o1(new_n254));
  norp02aa1n02x5               g159(.a(new_n254), .b(new_n251), .o1(\s[24] ));
  nor042aa1n03x5               g160(.a(new_n250), .b(new_n244), .o1(new_n256));
  nano22aa1n06x5               g161(.a(new_n220), .b(new_n238), .c(new_n256), .out0(new_n257));
  tech160nm_fioai012aa1n05x5   g162(.a(new_n257), .b(new_n195), .c(new_n197), .o1(new_n258));
  inv000aa1n06x5               g163(.a(new_n224), .o1(new_n259));
  aoai13aa1n06x5               g164(.a(new_n238), .b(new_n259), .c(new_n219), .d(new_n215), .o1(new_n260));
  inv040aa1n02x5               g165(.a(new_n256), .o1(new_n261));
  oao003aa1n02x5               g166(.a(\a[24] ), .b(\b[23] ), .c(new_n248), .carry(new_n262));
  aoai13aa1n12x5               g167(.a(new_n262), .b(new_n261), .c(new_n260), .d(new_n241), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n263), .o1(new_n264));
  xnrc02aa1n12x5               g169(.a(\b[24] ), .b(\a[25] ), .out0(new_n265));
  inv000aa1d42x5               g170(.a(new_n265), .o1(new_n266));
  xnbna2aa1n03x5               g171(.a(new_n266), .b(new_n258), .c(new_n264), .out0(\s[25] ));
  nor042aa1n03x5               g172(.a(\b[24] ), .b(\a[25] ), .o1(new_n268));
  inv000aa1d42x5               g173(.a(new_n268), .o1(new_n269));
  tech160nm_fiaoi012aa1n04x5   g174(.a(new_n265), .b(new_n258), .c(new_n264), .o1(new_n270));
  xnrc02aa1n02x5               g175(.a(\b[25] ), .b(\a[26] ), .out0(new_n271));
  nano22aa1n03x7               g176(.a(new_n270), .b(new_n269), .c(new_n271), .out0(new_n272));
  aoai13aa1n03x5               g177(.a(new_n266), .b(new_n263), .c(new_n187), .d(new_n257), .o1(new_n273));
  aoi012aa1n03x5               g178(.a(new_n271), .b(new_n273), .c(new_n269), .o1(new_n274));
  nor002aa1n02x5               g179(.a(new_n274), .b(new_n272), .o1(\s[26] ));
  nor042aa1n09x5               g180(.a(new_n271), .b(new_n265), .o1(new_n276));
  nano32aa1n06x5               g181(.a(new_n220), .b(new_n276), .c(new_n238), .d(new_n256), .out0(new_n277));
  oai012aa1n18x5               g182(.a(new_n277), .b(new_n195), .c(new_n197), .o1(new_n278));
  oao003aa1n02x5               g183(.a(\a[26] ), .b(\b[25] ), .c(new_n269), .carry(new_n279));
  aobi12aa1n18x5               g184(.a(new_n279), .b(new_n263), .c(new_n276), .out0(new_n280));
  xorc02aa1n12x5               g185(.a(\a[27] ), .b(\b[26] ), .out0(new_n281));
  xnbna2aa1n03x5               g186(.a(new_n281), .b(new_n278), .c(new_n280), .out0(\s[27] ));
  norp02aa1n02x5               g187(.a(\b[26] ), .b(\a[27] ), .o1(new_n283));
  inv040aa1n03x5               g188(.a(new_n283), .o1(new_n284));
  aobi12aa1n06x5               g189(.a(new_n281), .b(new_n278), .c(new_n280), .out0(new_n285));
  xnrc02aa1n02x5               g190(.a(\b[27] ), .b(\a[28] ), .out0(new_n286));
  nano22aa1n03x7               g191(.a(new_n285), .b(new_n284), .c(new_n286), .out0(new_n287));
  aoai13aa1n03x5               g192(.a(new_n256), .b(new_n242), .c(new_n225), .d(new_n238), .o1(new_n288));
  inv000aa1d42x5               g193(.a(new_n276), .o1(new_n289));
  aoai13aa1n04x5               g194(.a(new_n279), .b(new_n289), .c(new_n288), .d(new_n262), .o1(new_n290));
  aoai13aa1n02x7               g195(.a(new_n281), .b(new_n290), .c(new_n187), .d(new_n277), .o1(new_n291));
  aoi012aa1n02x5               g196(.a(new_n286), .b(new_n291), .c(new_n284), .o1(new_n292));
  norp02aa1n03x5               g197(.a(new_n292), .b(new_n287), .o1(\s[28] ));
  norb02aa1n02x5               g198(.a(new_n281), .b(new_n286), .out0(new_n294));
  aobi12aa1n06x5               g199(.a(new_n294), .b(new_n278), .c(new_n280), .out0(new_n295));
  oao003aa1n02x5               g200(.a(\a[28] ), .b(\b[27] ), .c(new_n284), .carry(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[28] ), .b(\a[29] ), .out0(new_n297));
  nano22aa1n03x7               g202(.a(new_n295), .b(new_n296), .c(new_n297), .out0(new_n298));
  aoai13aa1n02x7               g203(.a(new_n294), .b(new_n290), .c(new_n187), .d(new_n277), .o1(new_n299));
  aoi012aa1n03x5               g204(.a(new_n297), .b(new_n299), .c(new_n296), .o1(new_n300));
  norp02aa1n03x5               g205(.a(new_n300), .b(new_n298), .o1(\s[29] ));
  xorb03aa1n02x5               g206(.a(new_n112), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g207(.a(new_n281), .b(new_n297), .c(new_n286), .out0(new_n303));
  aobi12aa1n06x5               g208(.a(new_n303), .b(new_n278), .c(new_n280), .out0(new_n304));
  oao003aa1n02x5               g209(.a(\a[29] ), .b(\b[28] ), .c(new_n296), .carry(new_n305));
  xnrc02aa1n02x5               g210(.a(\b[29] ), .b(\a[30] ), .out0(new_n306));
  nano22aa1n03x7               g211(.a(new_n304), .b(new_n305), .c(new_n306), .out0(new_n307));
  aoai13aa1n02x5               g212(.a(new_n303), .b(new_n290), .c(new_n187), .d(new_n277), .o1(new_n308));
  tech160nm_fiaoi012aa1n02p5x5 g213(.a(new_n306), .b(new_n308), .c(new_n305), .o1(new_n309));
  norp02aa1n03x5               g214(.a(new_n309), .b(new_n307), .o1(\s[30] ));
  norb02aa1n02x5               g215(.a(new_n303), .b(new_n306), .out0(new_n311));
  aobi12aa1n06x5               g216(.a(new_n311), .b(new_n278), .c(new_n280), .out0(new_n312));
  oao003aa1n02x5               g217(.a(\a[30] ), .b(\b[29] ), .c(new_n305), .carry(new_n313));
  xnrc02aa1n02x5               g218(.a(\b[30] ), .b(\a[31] ), .out0(new_n314));
  nano22aa1n03x7               g219(.a(new_n312), .b(new_n313), .c(new_n314), .out0(new_n315));
  aoai13aa1n02x7               g220(.a(new_n311), .b(new_n290), .c(new_n187), .d(new_n277), .o1(new_n316));
  aoi012aa1n02x5               g221(.a(new_n314), .b(new_n316), .c(new_n313), .o1(new_n317));
  norp02aa1n03x5               g222(.a(new_n317), .b(new_n315), .o1(\s[31] ));
  xnrb03aa1n02x5               g223(.a(new_n114), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g224(.a(\a[3] ), .b(\b[2] ), .c(new_n114), .o1(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g226(.a(new_n121), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  orn002aa1n02x5               g227(.a(\a[5] ), .b(\b[4] ), .o(new_n323));
  nanb02aa1n02x5               g228(.a(new_n109), .b(new_n121), .out0(new_n324));
  xobna2aa1n03x5               g229(.a(new_n108), .b(new_n324), .c(new_n323), .out0(\s[6] ));
  norp02aa1n02x5               g230(.a(new_n109), .b(new_n108), .o1(new_n326));
  aobi12aa1n02x5               g231(.a(new_n125), .b(new_n121), .c(new_n326), .out0(new_n327));
  xnrb03aa1n02x5               g232(.a(new_n327), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g233(.a(\a[7] ), .b(\b[6] ), .c(new_n327), .o1(new_n329));
  xorb03aa1n02x5               g234(.a(new_n329), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g235(.a(new_n152), .b(new_n100), .c(new_n102), .out0(\s[9] ));
endmodule


