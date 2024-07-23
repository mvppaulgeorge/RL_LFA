// Benchmark "adder" written by ABC on Thu Jul 18 12:26:50 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n166, new_n167, new_n168, new_n169, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n309,
    new_n311, new_n312, new_n315, new_n316, new_n317, new_n320, new_n322;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xorc02aa1n02x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  inv000aa1d42x5               g002(.a(\a[9] ), .o1(new_n98));
  nanb02aa1n02x5               g003(.a(\b[8] ), .b(new_n98), .out0(new_n99));
  tech160nm_finor002aa1n03p5x5 g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand22aa1n03x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  nand22aa1n03x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  tech160nm_fiaoi012aa1n05x5   g007(.a(new_n100), .b(new_n101), .c(new_n102), .o1(new_n103));
  norp02aa1n06x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nanp02aa1n04x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nor002aa1d32x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nand42aa1n03x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nona23aa1n09x5               g012(.a(new_n107), .b(new_n105), .c(new_n104), .d(new_n106), .out0(new_n108));
  tech160nm_fioai012aa1n03p5x5 g013(.a(new_n105), .b(new_n106), .c(new_n104), .o1(new_n109));
  oai012aa1n12x5               g014(.a(new_n109), .b(new_n108), .c(new_n103), .o1(new_n110));
  nor042aa1n04x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  nand42aa1n16x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  norb02aa1n03x4               g017(.a(new_n112), .b(new_n111), .out0(new_n113));
  nor002aa1n03x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nand42aa1d28x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  norb02aa1n03x4               g020(.a(new_n115), .b(new_n114), .out0(new_n116));
  tech160nm_fixnrc02aa1n04x5   g021(.a(\b[6] ), .b(\a[7] ), .out0(new_n117));
  tech160nm_fixnrc02aa1n04x5   g022(.a(\b[7] ), .b(\a[8] ), .out0(new_n118));
  nano23aa1n06x5               g023(.a(new_n118), .b(new_n117), .c(new_n116), .d(new_n113), .out0(new_n119));
  inv040aa1d32x5               g024(.a(\a[7] ), .o1(new_n120));
  inv040aa1d32x5               g025(.a(\b[6] ), .o1(new_n121));
  nanp02aa1n02x5               g026(.a(new_n121), .b(new_n120), .o1(new_n122));
  and002aa1n02x5               g027(.a(\b[7] ), .b(\a[8] ), .o(new_n123));
  orn002aa1n02x5               g028(.a(\a[8] ), .b(\b[7] ), .o(new_n124));
  oai022aa1d18x5               g029(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n125));
  oai112aa1n02x5               g030(.a(new_n125), .b(new_n115), .c(new_n121), .d(new_n120), .o1(new_n126));
  aoai13aa1n06x5               g031(.a(new_n124), .b(new_n123), .c(new_n126), .d(new_n122), .o1(new_n127));
  xorc02aa1n02x5               g032(.a(\a[9] ), .b(\b[8] ), .out0(new_n128));
  aoai13aa1n02x5               g033(.a(new_n128), .b(new_n127), .c(new_n119), .d(new_n110), .o1(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n97), .b(new_n129), .c(new_n99), .out0(\s[10] ));
  oai022aa1d24x5               g035(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n131));
  inv000aa1d42x5               g036(.a(new_n131), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(new_n129), .b(new_n132), .o1(new_n133));
  nor042aa1n12x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  aoi022aa1d24x5               g039(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  xnrc02aa1n02x5               g041(.a(\b[10] ), .b(\a[11] ), .out0(new_n137));
  aob012aa1n02x5               g042(.a(new_n133), .b(\b[9] ), .c(\a[10] ), .out0(new_n138));
  aoi022aa1n02x5               g043(.a(new_n138), .b(new_n137), .c(new_n133), .d(new_n136), .o1(\s[11] ));
  nor042aa1n06x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nand42aa1n10x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nanb02aa1n02x5               g046(.a(new_n140), .b(new_n141), .out0(new_n142));
  aoai13aa1n02x5               g047(.a(new_n142), .b(new_n134), .c(new_n133), .d(new_n136), .o1(new_n143));
  norb03aa1d15x5               g048(.a(new_n141), .b(new_n134), .c(new_n140), .out0(new_n144));
  aob012aa1n02x5               g049(.a(new_n144), .b(new_n133), .c(new_n136), .out0(new_n145));
  nanp02aa1n02x5               g050(.a(new_n143), .b(new_n145), .o1(\s[12] ));
  nona23aa1d18x5               g051(.a(new_n131), .b(new_n135), .c(new_n140), .d(new_n134), .out0(new_n147));
  nanp02aa1n02x5               g052(.a(\b[8] ), .b(\a[9] ), .o1(new_n148));
  nano22aa1n03x5               g053(.a(new_n134), .b(new_n135), .c(new_n148), .out0(new_n149));
  and003aa1n02x5               g054(.a(new_n149), .b(new_n147), .c(new_n144), .o(new_n150));
  aoai13aa1n06x5               g055(.a(new_n150), .b(new_n127), .c(new_n119), .d(new_n110), .o1(new_n151));
  aoi022aa1d24x5               g056(.a(new_n147), .b(new_n144), .c(\a[12] ), .d(\b[11] ), .o1(new_n152));
  inv000aa1d42x5               g057(.a(new_n152), .o1(new_n153));
  nor042aa1d18x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nand42aa1d28x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  norb02aa1n02x5               g060(.a(new_n155), .b(new_n154), .out0(new_n156));
  xnbna2aa1n03x5               g061(.a(new_n156), .b(new_n151), .c(new_n153), .out0(\s[13] ));
  inv000aa1d42x5               g062(.a(new_n154), .o1(new_n158));
  aob012aa1n02x5               g063(.a(new_n156), .b(new_n151), .c(new_n153), .out0(new_n159));
  nor042aa1n06x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nand42aa1d28x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n161), .b(new_n160), .out0(new_n162));
  oaih22aa1d12x5               g067(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n163));
  nanb03aa1n02x5               g068(.a(new_n163), .b(new_n159), .c(new_n161), .out0(new_n164));
  aoai13aa1n02x5               g069(.a(new_n164), .b(new_n162), .c(new_n158), .d(new_n159), .o1(\s[14] ));
  nano23aa1d15x5               g070(.a(new_n154), .b(new_n160), .c(new_n161), .d(new_n155), .out0(new_n166));
  inv000aa1d42x5               g071(.a(new_n166), .o1(new_n167));
  oai012aa1n02x5               g072(.a(new_n161), .b(new_n160), .c(new_n154), .o1(new_n168));
  aoai13aa1n06x5               g073(.a(new_n168), .b(new_n167), .c(new_n151), .d(new_n153), .o1(new_n169));
  xorb03aa1n02x5               g074(.a(new_n169), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  xorc02aa1n12x5               g075(.a(\a[16] ), .b(\b[15] ), .out0(new_n171));
  nor042aa1n02x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  nanp02aa1n04x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  aoi012aa1n02x5               g078(.a(new_n172), .b(new_n169), .c(new_n173), .o1(new_n174));
  norb02aa1n06x4               g079(.a(new_n173), .b(new_n172), .out0(new_n175));
  norb02aa1n02x5               g080(.a(new_n171), .b(new_n172), .out0(new_n176));
  aob012aa1n02x5               g081(.a(new_n176), .b(new_n169), .c(new_n175), .out0(new_n177));
  oaih12aa1n02x5               g082(.a(new_n177), .b(new_n174), .c(new_n171), .o1(\s[16] ));
  nand23aa1n06x5               g083(.a(new_n166), .b(new_n175), .c(new_n171), .o1(new_n179));
  nano32aa1n03x7               g084(.a(new_n179), .b(new_n149), .c(new_n147), .d(new_n144), .out0(new_n180));
  aoai13aa1n12x5               g085(.a(new_n180), .b(new_n127), .c(new_n110), .d(new_n119), .o1(new_n181));
  aoi013aa1n02x4               g086(.a(new_n172), .b(new_n163), .c(new_n161), .d(new_n173), .o1(new_n182));
  oaoi03aa1n02x5               g087(.a(\a[16] ), .b(\b[15] ), .c(new_n182), .o1(new_n183));
  aoib12aa1n12x5               g088(.a(new_n183), .b(new_n152), .c(new_n179), .out0(new_n184));
  xorc02aa1n12x5               g089(.a(\a[17] ), .b(\b[16] ), .out0(new_n185));
  xnbna2aa1n03x5               g090(.a(new_n185), .b(new_n181), .c(new_n184), .out0(\s[17] ));
  aobi12aa1n02x5               g091(.a(new_n185), .b(new_n181), .c(new_n184), .out0(new_n187));
  xorc02aa1n12x5               g092(.a(\a[18] ), .b(\b[17] ), .out0(new_n188));
  nand42aa1n08x5               g093(.a(new_n181), .b(new_n184), .o1(new_n189));
  orn002aa1n02x5               g094(.a(\a[17] ), .b(\b[16] ), .o(new_n190));
  aobi12aa1n02x5               g095(.a(new_n190), .b(new_n189), .c(new_n185), .out0(new_n191));
  oai022aa1n04x7               g096(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n192));
  tech160nm_fiao0012aa1n02p5x5 g097(.a(new_n192), .b(\a[18] ), .c(\b[17] ), .o(new_n193));
  oai022aa1n02x5               g098(.a(new_n191), .b(new_n188), .c(new_n193), .d(new_n187), .o1(\s[18] ));
  nanp02aa1n02x5               g099(.a(new_n188), .b(new_n185), .o1(new_n195));
  aob012aa1n02x5               g100(.a(new_n192), .b(\b[17] ), .c(\a[18] ), .out0(new_n196));
  aoai13aa1n04x5               g101(.a(new_n196), .b(new_n195), .c(new_n181), .d(new_n184), .o1(new_n197));
  xorb03aa1n02x5               g102(.a(new_n197), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n04x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nand22aa1n06x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  nor042aa1n04x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  nand02aa1d04x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  nanb02aa1n02x5               g108(.a(new_n202), .b(new_n203), .out0(new_n204));
  aoai13aa1n02x5               g109(.a(new_n204), .b(new_n200), .c(new_n197), .d(new_n201), .o1(new_n205));
  aoi112aa1n03x4               g110(.a(new_n200), .b(new_n204), .c(new_n197), .d(new_n201), .o1(new_n206));
  nanb02aa1n03x5               g111(.a(new_n206), .b(new_n205), .out0(\s[20] ));
  nano23aa1n06x5               g112(.a(new_n200), .b(new_n202), .c(new_n203), .d(new_n201), .out0(new_n208));
  nanp03aa1d12x5               g113(.a(new_n208), .b(new_n185), .c(new_n188), .o1(new_n209));
  inv000aa1d42x5               g114(.a(new_n209), .o1(new_n210));
  nona23aa1n02x4               g115(.a(new_n203), .b(new_n201), .c(new_n200), .d(new_n202), .out0(new_n211));
  oaih12aa1n02x5               g116(.a(new_n203), .b(new_n202), .c(new_n200), .o1(new_n212));
  oai012aa1n04x7               g117(.a(new_n212), .b(new_n211), .c(new_n196), .o1(new_n213));
  xorc02aa1n02x5               g118(.a(\a[21] ), .b(\b[20] ), .out0(new_n214));
  aoai13aa1n06x5               g119(.a(new_n214), .b(new_n213), .c(new_n189), .d(new_n210), .o1(new_n215));
  aoi112aa1n02x5               g120(.a(new_n214), .b(new_n213), .c(new_n189), .d(new_n210), .o1(new_n216));
  norb02aa1n02x7               g121(.a(new_n215), .b(new_n216), .out0(\s[21] ));
  nor042aa1n06x5               g122(.a(\b[20] ), .b(\a[21] ), .o1(new_n218));
  inv040aa1n03x5               g123(.a(new_n218), .o1(new_n219));
  xorc02aa1n02x5               g124(.a(\a[22] ), .b(\b[21] ), .out0(new_n220));
  inv040aa1d32x5               g125(.a(\a[22] ), .o1(new_n221));
  inv000aa1d42x5               g126(.a(\b[21] ), .o1(new_n222));
  aoi012aa1n02x5               g127(.a(new_n218), .b(new_n221), .c(new_n222), .o1(new_n223));
  oai112aa1n02x7               g128(.a(new_n215), .b(new_n223), .c(new_n222), .d(new_n221), .o1(new_n224));
  aoai13aa1n03x5               g129(.a(new_n224), .b(new_n220), .c(new_n219), .d(new_n215), .o1(\s[22] ));
  nano32aa1n02x4               g130(.a(new_n195), .b(new_n220), .c(new_n208), .d(new_n214), .out0(new_n226));
  oaoi03aa1n02x5               g131(.a(\a[18] ), .b(\b[17] ), .c(new_n190), .o1(new_n227));
  inv000aa1n02x5               g132(.a(new_n212), .o1(new_n228));
  inv000aa1d42x5               g133(.a(\a[21] ), .o1(new_n229));
  xroi22aa1d04x5               g134(.a(new_n229), .b(\b[20] ), .c(new_n221), .d(\b[21] ), .out0(new_n230));
  aoai13aa1n03x5               g135(.a(new_n230), .b(new_n228), .c(new_n208), .d(new_n227), .o1(new_n231));
  oaoi03aa1n09x5               g136(.a(\a[22] ), .b(\b[21] ), .c(new_n219), .o1(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  nanp02aa1n02x5               g138(.a(new_n231), .b(new_n233), .o1(new_n234));
  xnrc02aa1n12x5               g139(.a(\b[22] ), .b(\a[23] ), .out0(new_n235));
  inv000aa1d42x5               g140(.a(new_n235), .o1(new_n236));
  aoai13aa1n06x5               g141(.a(new_n236), .b(new_n234), .c(new_n189), .d(new_n226), .o1(new_n237));
  aoi112aa1n02x5               g142(.a(new_n236), .b(new_n234), .c(new_n189), .d(new_n226), .o1(new_n238));
  norb02aa1n02x7               g143(.a(new_n237), .b(new_n238), .out0(\s[23] ));
  nor002aa1n03x5               g144(.a(\b[22] ), .b(\a[23] ), .o1(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  xorc02aa1n02x5               g146(.a(\a[24] ), .b(\b[23] ), .out0(new_n242));
  inv000aa1d42x5               g147(.a(\a[24] ), .o1(new_n243));
  inv000aa1d42x5               g148(.a(\b[23] ), .o1(new_n244));
  aoi012aa1n02x5               g149(.a(new_n240), .b(new_n243), .c(new_n244), .o1(new_n245));
  oai112aa1n02x7               g150(.a(new_n237), .b(new_n245), .c(new_n244), .d(new_n243), .o1(new_n246));
  aoai13aa1n03x5               g151(.a(new_n246), .b(new_n242), .c(new_n241), .d(new_n237), .o1(\s[24] ));
  norb02aa1n03x5               g152(.a(new_n242), .b(new_n235), .out0(new_n248));
  nano22aa1n02x4               g153(.a(new_n209), .b(new_n248), .c(new_n230), .out0(new_n249));
  inv000aa1n02x5               g154(.a(new_n248), .o1(new_n250));
  oaoi03aa1n02x5               g155(.a(new_n243), .b(new_n244), .c(new_n240), .o1(new_n251));
  aoai13aa1n03x5               g156(.a(new_n251), .b(new_n250), .c(new_n231), .d(new_n233), .o1(new_n252));
  xorc02aa1n12x5               g157(.a(\a[25] ), .b(\b[24] ), .out0(new_n253));
  aoai13aa1n06x5               g158(.a(new_n253), .b(new_n252), .c(new_n189), .d(new_n249), .o1(new_n254));
  aoi112aa1n02x5               g159(.a(new_n253), .b(new_n252), .c(new_n189), .d(new_n249), .o1(new_n255));
  norb02aa1n03x4               g160(.a(new_n254), .b(new_n255), .out0(\s[25] ));
  nor042aa1n02x5               g161(.a(\b[24] ), .b(\a[25] ), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n257), .o1(new_n258));
  tech160nm_fixorc02aa1n04x5   g163(.a(\a[26] ), .b(\b[25] ), .out0(new_n259));
  nanp02aa1n02x5               g164(.a(\b[25] ), .b(\a[26] ), .o1(new_n260));
  inv000aa1d42x5               g165(.a(\a[26] ), .o1(new_n261));
  inv000aa1d42x5               g166(.a(\b[25] ), .o1(new_n262));
  aoi012aa1n02x5               g167(.a(new_n257), .b(new_n261), .c(new_n262), .o1(new_n263));
  nand43aa1n03x5               g168(.a(new_n254), .b(new_n260), .c(new_n263), .o1(new_n264));
  aoai13aa1n03x5               g169(.a(new_n264), .b(new_n259), .c(new_n258), .d(new_n254), .o1(\s[26] ));
  nand42aa1n04x5               g170(.a(new_n259), .b(new_n253), .o1(new_n266));
  nano23aa1n03x7               g171(.a(new_n209), .b(new_n266), .c(new_n248), .d(new_n230), .out0(new_n267));
  nand22aa1n03x5               g172(.a(new_n189), .b(new_n267), .o1(new_n268));
  inv000aa1n02x5               g173(.a(new_n263), .o1(new_n269));
  inv000aa1d42x5               g174(.a(new_n266), .o1(new_n270));
  aoi022aa1n02x5               g175(.a(new_n252), .b(new_n270), .c(new_n260), .d(new_n269), .o1(new_n271));
  xorc02aa1n02x5               g176(.a(\a[27] ), .b(\b[26] ), .out0(new_n272));
  xnbna2aa1n03x5               g177(.a(new_n272), .b(new_n268), .c(new_n271), .out0(\s[27] ));
  xorc02aa1n02x5               g178(.a(\a[28] ), .b(\b[27] ), .out0(new_n274));
  aobi12aa1n06x5               g179(.a(new_n267), .b(new_n181), .c(new_n184), .out0(new_n275));
  aoai13aa1n03x5               g180(.a(new_n248), .b(new_n232), .c(new_n213), .d(new_n230), .o1(new_n276));
  oaoi03aa1n02x5               g181(.a(new_n261), .b(new_n262), .c(new_n257), .o1(new_n277));
  aoai13aa1n04x5               g182(.a(new_n277), .b(new_n266), .c(new_n276), .d(new_n251), .o1(new_n278));
  norp02aa1n02x5               g183(.a(\b[26] ), .b(\a[27] ), .o1(new_n279));
  oaoi13aa1n03x5               g184(.a(new_n279), .b(new_n272), .c(new_n278), .d(new_n275), .o1(new_n280));
  oaih12aa1n02x5               g185(.a(new_n272), .b(new_n278), .c(new_n275), .o1(new_n281));
  oai112aa1n02x7               g186(.a(new_n281), .b(new_n274), .c(\b[26] ), .d(\a[27] ), .o1(new_n282));
  oaih12aa1n02x5               g187(.a(new_n282), .b(new_n280), .c(new_n274), .o1(\s[28] ));
  and002aa1n02x5               g188(.a(new_n274), .b(new_n272), .o(new_n284));
  oaih12aa1n02x5               g189(.a(new_n284), .b(new_n278), .c(new_n275), .o1(new_n285));
  orn002aa1n03x5               g190(.a(\a[27] ), .b(\b[26] ), .o(new_n286));
  oao003aa1n02x5               g191(.a(\a[28] ), .b(\b[27] ), .c(new_n286), .carry(new_n287));
  tech160nm_finand02aa1n03p5x5 g192(.a(new_n285), .b(new_n287), .o1(new_n288));
  xorc02aa1n02x5               g193(.a(\a[29] ), .b(\b[28] ), .out0(new_n289));
  norb02aa1n02x5               g194(.a(new_n287), .b(new_n289), .out0(new_n290));
  aoi022aa1n02x7               g195(.a(new_n288), .b(new_n289), .c(new_n285), .d(new_n290), .o1(\s[29] ));
  xorb03aa1n02x5               g196(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g197(.a(new_n272), .b(new_n289), .c(new_n274), .o(new_n293));
  oaih12aa1n02x5               g198(.a(new_n293), .b(new_n278), .c(new_n275), .o1(new_n294));
  oaoi03aa1n09x5               g199(.a(\a[29] ), .b(\b[28] ), .c(new_n287), .o1(new_n295));
  inv000aa1d42x5               g200(.a(new_n295), .o1(new_n296));
  tech160nm_finand02aa1n03p5x5 g201(.a(new_n294), .b(new_n296), .o1(new_n297));
  xorc02aa1n02x5               g202(.a(\a[30] ), .b(\b[29] ), .out0(new_n298));
  norp02aa1n02x5               g203(.a(new_n295), .b(new_n298), .o1(new_n299));
  aoi022aa1n02x7               g204(.a(new_n297), .b(new_n298), .c(new_n294), .d(new_n299), .o1(\s[30] ));
  xnrc02aa1n02x5               g205(.a(\b[30] ), .b(\a[31] ), .out0(new_n301));
  and003aa1n02x5               g206(.a(new_n284), .b(new_n298), .c(new_n289), .o(new_n302));
  oaih12aa1n02x5               g207(.a(new_n302), .b(new_n278), .c(new_n275), .o1(new_n303));
  oao003aa1n02x5               g208(.a(\a[30] ), .b(\b[29] ), .c(new_n296), .carry(new_n304));
  tech160nm_fiaoi012aa1n02p5x5 g209(.a(new_n301), .b(new_n303), .c(new_n304), .o1(new_n305));
  aobi12aa1n02x7               g210(.a(new_n302), .b(new_n268), .c(new_n271), .out0(new_n306));
  nano22aa1n03x5               g211(.a(new_n306), .b(new_n301), .c(new_n304), .out0(new_n307));
  norp02aa1n03x5               g212(.a(new_n305), .b(new_n307), .o1(\s[31] ));
  inv000aa1d42x5               g213(.a(new_n106), .o1(new_n309));
  xnbna2aa1n03x5               g214(.a(new_n103), .b(new_n107), .c(new_n309), .out0(\s[3] ));
  norb02aa1n02x5               g215(.a(new_n105), .b(new_n104), .out0(new_n311));
  nanb03aa1n02x5               g216(.a(new_n103), .b(new_n107), .c(new_n309), .out0(new_n312));
  xnbna2aa1n03x5               g217(.a(new_n311), .b(new_n312), .c(new_n309), .out0(\s[4] ));
  xorb03aa1n02x5               g218(.a(new_n110), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g219(.a(new_n111), .b(new_n110), .c(new_n112), .o1(new_n315));
  norb03aa1n02x5               g220(.a(new_n115), .b(new_n111), .c(new_n114), .out0(new_n316));
  aob012aa1n02x5               g221(.a(new_n316), .b(new_n110), .c(new_n113), .out0(new_n317));
  oai012aa1n02x5               g222(.a(new_n317), .b(new_n315), .c(new_n116), .o1(\s[6] ));
  xnbna2aa1n03x5               g223(.a(new_n117), .b(new_n317), .c(new_n115), .out0(\s[7] ));
  nanb03aa1n02x5               g224(.a(new_n117), .b(new_n317), .c(new_n115), .out0(new_n320));
  xobna2aa1n03x5               g225(.a(new_n118), .b(new_n320), .c(new_n122), .out0(\s[8] ));
  aoi112aa1n02x5               g226(.a(new_n127), .b(new_n128), .c(new_n119), .d(new_n110), .o1(new_n322));
  norb02aa1n02x5               g227(.a(new_n129), .b(new_n322), .out0(\s[9] ));
endmodule


