// Benchmark "adder" written by ABC on Wed Jul 17 13:17:53 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n131,
    new_n132, new_n133, new_n134, new_n135, new_n136, new_n137, new_n138,
    new_n139, new_n141, new_n142, new_n143, new_n144, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n151, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n163,
    new_n164, new_n165, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n195,
    new_n196, new_n197, new_n198, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n220,
    new_n221, new_n222, new_n223, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n308, new_n310, new_n312,
    new_n313, new_n316;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xnrc02aa1n12x5               g001(.a(\b[9] ), .b(\a[10] ), .out0(new_n97));
  nor002aa1d32x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  nand02aa1d06x5               g003(.a(\b[8] ), .b(\a[9] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[3] ), .b(\a[4] ), .o1(new_n100));
  nand02aa1n03x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  nand02aa1n04x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nor042aa1n02x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  nona22aa1n03x5               g008(.a(new_n102), .b(new_n103), .c(new_n101), .out0(new_n104));
  nor042aa1n06x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand42aa1n04x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nano22aa1n03x7               g011(.a(new_n105), .b(new_n102), .c(new_n106), .out0(new_n107));
  oab012aa1n04x5               g012(.a(new_n105), .b(\a[4] ), .c(\b[3] ), .out0(new_n108));
  inv000aa1n02x5               g013(.a(new_n108), .o1(new_n109));
  aoai13aa1n06x5               g014(.a(new_n100), .b(new_n109), .c(new_n107), .d(new_n104), .o1(new_n110));
  nor042aa1n04x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nanp02aa1n09x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nor042aa1n06x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nanp02aa1n04x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nano23aa1n06x5               g019(.a(new_n111), .b(new_n113), .c(new_n114), .d(new_n112), .out0(new_n115));
  nor002aa1n02x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nand42aa1d28x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  norb02aa1n02x5               g022(.a(new_n117), .b(new_n116), .out0(new_n118));
  xnrc02aa1n12x5               g023(.a(\b[4] ), .b(\a[5] ), .out0(new_n119));
  inv000aa1d42x5               g024(.a(new_n119), .o1(new_n120));
  nanp03aa1n02x5               g025(.a(new_n120), .b(new_n115), .c(new_n118), .o1(new_n121));
  orn002aa1n03x5               g026(.a(\a[6] ), .b(\b[5] ), .o(new_n122));
  oai112aa1n06x5               g027(.a(new_n122), .b(new_n117), .c(\b[4] ), .d(\a[5] ), .o1(new_n123));
  inv000aa1n03x5               g028(.a(new_n113), .o1(new_n124));
  oaoi03aa1n03x5               g029(.a(\a[8] ), .b(\b[7] ), .c(new_n124), .o1(new_n125));
  aoi013aa1n02x4               g030(.a(new_n125), .b(new_n115), .c(new_n117), .d(new_n123), .o1(new_n126));
  oai012aa1n09x5               g031(.a(new_n126), .b(new_n110), .c(new_n121), .o1(new_n127));
  aoai13aa1n02x5               g032(.a(new_n97), .b(new_n98), .c(new_n127), .d(new_n99), .o1(new_n128));
  norb02aa1n09x5               g033(.a(new_n106), .b(new_n105), .out0(new_n129));
  oai112aa1n06x5               g034(.a(new_n129), .b(new_n102), .c(new_n103), .d(new_n101), .o1(new_n130));
  aoi022aa1n09x5               g035(.a(new_n130), .b(new_n108), .c(\b[3] ), .d(\a[4] ), .o1(new_n131));
  norb02aa1n02x7               g036(.a(new_n112), .b(new_n111), .out0(new_n132));
  nanb02aa1n03x5               g037(.a(new_n113), .b(new_n114), .out0(new_n133));
  nano23aa1n06x5               g038(.a(new_n119), .b(new_n133), .c(new_n132), .d(new_n118), .out0(new_n134));
  nand43aa1n02x5               g039(.a(new_n115), .b(new_n117), .c(new_n123), .o1(new_n135));
  inv000aa1n02x5               g040(.a(new_n125), .o1(new_n136));
  nand22aa1n03x5               g041(.a(new_n135), .b(new_n136), .o1(new_n137));
  aoai13aa1n02x5               g042(.a(new_n99), .b(new_n137), .c(new_n131), .d(new_n134), .o1(new_n138));
  nona22aa1n02x4               g043(.a(new_n138), .b(new_n98), .c(new_n97), .out0(new_n139));
  nanp02aa1n02x5               g044(.a(new_n128), .b(new_n139), .o1(\s[10] ));
  nanp02aa1n02x5               g045(.a(\b[9] ), .b(\a[10] ), .o1(new_n141));
  nand42aa1d28x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  norp02aa1n12x5               g047(.a(\b[10] ), .b(\a[11] ), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n142), .b(new_n143), .out0(new_n144));
  xobna2aa1n03x5               g049(.a(new_n144), .b(new_n139), .c(new_n141), .out0(\s[11] ));
  aoi013aa1n02x4               g050(.a(new_n143), .b(new_n139), .c(new_n141), .d(new_n144), .o1(new_n146));
  nor042aa1n06x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  nand42aa1d28x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  norb02aa1n02x5               g053(.a(new_n148), .b(new_n147), .out0(new_n149));
  nona22aa1n09x5               g054(.a(new_n148), .b(new_n147), .c(new_n143), .out0(new_n150));
  aoi013aa1n02x4               g055(.a(new_n150), .b(new_n139), .c(new_n144), .d(new_n141), .o1(new_n151));
  oabi12aa1n02x5               g056(.a(new_n151), .b(new_n146), .c(new_n149), .out0(\s[12] ));
  nano23aa1d15x5               g057(.a(new_n147), .b(new_n143), .c(new_n148), .d(new_n142), .out0(new_n153));
  nanb02aa1n09x5               g058(.a(new_n98), .b(new_n99), .out0(new_n154));
  nona22aa1d30x5               g059(.a(new_n153), .b(new_n97), .c(new_n154), .out0(new_n155));
  inv000aa1d42x5               g060(.a(new_n155), .o1(new_n156));
  aoai13aa1n06x5               g061(.a(new_n156), .b(new_n137), .c(new_n131), .d(new_n134), .o1(new_n157));
  inv000aa1n03x5               g062(.a(new_n98), .o1(new_n158));
  oaoi03aa1n12x5               g063(.a(\a[10] ), .b(\b[9] ), .c(new_n158), .o1(new_n159));
  aoi022aa1n12x5               g064(.a(new_n153), .b(new_n159), .c(new_n150), .d(new_n148), .o1(new_n160));
  xnrc02aa1n12x5               g065(.a(\b[12] ), .b(\a[13] ), .out0(new_n161));
  xobna2aa1n03x5               g066(.a(new_n161), .b(new_n157), .c(new_n160), .out0(\s[13] ));
  orn002aa1n02x5               g067(.a(\a[13] ), .b(\b[12] ), .o(new_n163));
  tech160nm_fiao0012aa1n02p5x5 g068(.a(new_n161), .b(new_n157), .c(new_n160), .o(new_n164));
  xnrc02aa1n12x5               g069(.a(\b[13] ), .b(\a[14] ), .out0(new_n165));
  xobna2aa1n03x5               g070(.a(new_n165), .b(new_n164), .c(new_n163), .out0(\s[14] ));
  nor042aa1n12x5               g071(.a(new_n165), .b(new_n161), .o1(new_n167));
  inv000aa1d42x5               g072(.a(new_n167), .o1(new_n168));
  norp02aa1n03x5               g073(.a(\b[12] ), .b(\a[13] ), .o1(new_n169));
  norp02aa1n02x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  nanp02aa1n02x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  tech160nm_fioai012aa1n04x5   g076(.a(new_n171), .b(new_n170), .c(new_n169), .o1(new_n172));
  aoai13aa1n06x5               g077(.a(new_n172), .b(new_n168), .c(new_n157), .d(new_n160), .o1(new_n173));
  xorb03aa1n02x5               g078(.a(new_n173), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  xnrc02aa1n12x5               g079(.a(\b[14] ), .b(\a[15] ), .out0(new_n175));
  inv000aa1d42x5               g080(.a(new_n175), .o1(new_n176));
  norp02aa1n02x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  inv040aa1d32x5               g082(.a(\a[16] ), .o1(new_n178));
  inv030aa1d32x5               g083(.a(\b[15] ), .o1(new_n179));
  nand02aa1n03x5               g084(.a(new_n179), .b(new_n178), .o1(new_n180));
  nand02aa1n03x5               g085(.a(\b[15] ), .b(\a[16] ), .o1(new_n181));
  nand02aa1n03x5               g086(.a(new_n180), .b(new_n181), .o1(new_n182));
  aoai13aa1n03x5               g087(.a(new_n182), .b(new_n177), .c(new_n173), .d(new_n176), .o1(new_n183));
  oai112aa1n02x5               g088(.a(new_n180), .b(new_n181), .c(\b[14] ), .d(\a[15] ), .o1(new_n184));
  aoai13aa1n02x5               g089(.a(new_n183), .b(new_n184), .c(new_n176), .d(new_n173), .o1(\s[16] ));
  nor042aa1n06x5               g090(.a(new_n175), .b(new_n182), .o1(new_n186));
  nano22aa1d15x5               g091(.a(new_n155), .b(new_n167), .c(new_n186), .out0(new_n187));
  aoai13aa1n12x5               g092(.a(new_n187), .b(new_n137), .c(new_n131), .d(new_n134), .o1(new_n188));
  tech160nm_finand02aa1n03p5x5 g093(.a(new_n167), .b(new_n186), .o1(new_n189));
  oaoi03aa1n02x5               g094(.a(new_n178), .b(new_n179), .c(new_n177), .o1(new_n190));
  oai013aa1n09x5               g095(.a(new_n190), .b(new_n175), .c(new_n172), .d(new_n182), .o1(new_n191));
  oab012aa1d15x5               g096(.a(new_n191), .b(new_n160), .c(new_n189), .out0(new_n192));
  xorc02aa1n02x5               g097(.a(\a[17] ), .b(\b[16] ), .out0(new_n193));
  xnbna2aa1n03x5               g098(.a(new_n193), .b(new_n188), .c(new_n192), .out0(\s[17] ));
  inv000aa1d42x5               g099(.a(\a[18] ), .o1(new_n195));
  nand22aa1n03x5               g100(.a(new_n188), .b(new_n192), .o1(new_n196));
  norp02aa1n02x5               g101(.a(\b[16] ), .b(\a[17] ), .o1(new_n197));
  tech160nm_fiaoi012aa1n05x5   g102(.a(new_n197), .b(new_n196), .c(new_n193), .o1(new_n198));
  xorb03aa1n02x5               g103(.a(new_n198), .b(\b[17] ), .c(new_n195), .out0(\s[18] ));
  oabi12aa1n06x5               g104(.a(new_n191), .b(new_n160), .c(new_n189), .out0(new_n200));
  inv000aa1d42x5               g105(.a(\a[17] ), .o1(new_n201));
  xroi22aa1d04x5               g106(.a(new_n201), .b(\b[16] ), .c(new_n195), .d(\b[17] ), .out0(new_n202));
  aoai13aa1n06x5               g107(.a(new_n202), .b(new_n200), .c(new_n127), .d(new_n187), .o1(new_n203));
  nor042aa1n02x5               g108(.a(\b[17] ), .b(\a[18] ), .o1(new_n204));
  aoi112aa1n09x5               g109(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n205));
  norp02aa1n02x5               g110(.a(new_n205), .b(new_n204), .o1(new_n206));
  nor042aa1d18x5               g111(.a(\b[18] ), .b(\a[19] ), .o1(new_n207));
  nand22aa1n12x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  norb02aa1n15x5               g113(.a(new_n208), .b(new_n207), .out0(new_n209));
  xnbna2aa1n03x5               g114(.a(new_n209), .b(new_n203), .c(new_n206), .out0(\s[19] ));
  xnrc02aa1n02x5               g115(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aobi12aa1n02x5               g116(.a(new_n209), .b(new_n203), .c(new_n206), .out0(new_n212));
  nanp02aa1n03x5               g117(.a(new_n203), .b(new_n206), .o1(new_n213));
  aoi012aa1n02x5               g118(.a(new_n207), .b(new_n213), .c(new_n208), .o1(new_n214));
  nor042aa1n12x5               g119(.a(\b[19] ), .b(\a[20] ), .o1(new_n215));
  nand02aa1d28x5               g120(.a(\b[19] ), .b(\a[20] ), .o1(new_n216));
  norb02aa1n12x5               g121(.a(new_n216), .b(new_n215), .out0(new_n217));
  nona22aa1n02x4               g122(.a(new_n216), .b(new_n215), .c(new_n207), .out0(new_n218));
  oai022aa1n02x5               g123(.a(new_n214), .b(new_n217), .c(new_n218), .d(new_n212), .o1(\s[20] ));
  nano23aa1n09x5               g124(.a(new_n207), .b(new_n215), .c(new_n216), .d(new_n208), .out0(new_n220));
  nanp02aa1n02x5               g125(.a(new_n202), .b(new_n220), .o1(new_n221));
  aboi22aa1n03x5               g126(.a(new_n206), .b(new_n220), .c(new_n216), .d(new_n218), .out0(new_n222));
  aoai13aa1n04x5               g127(.a(new_n222), .b(new_n221), .c(new_n188), .d(new_n192), .o1(new_n223));
  xorb03aa1n02x5               g128(.a(new_n223), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  xorc02aa1n12x5               g129(.a(\a[21] ), .b(\b[20] ), .out0(new_n225));
  nor042aa1n09x5               g130(.a(\b[20] ), .b(\a[21] ), .o1(new_n226));
  xnrc02aa1n12x5               g131(.a(\b[21] ), .b(\a[22] ), .out0(new_n227));
  aoai13aa1n03x5               g132(.a(new_n227), .b(new_n226), .c(new_n223), .d(new_n225), .o1(new_n228));
  oabi12aa1n02x5               g133(.a(new_n227), .b(\a[21] ), .c(\b[20] ), .out0(new_n229));
  aoai13aa1n03x5               g134(.a(new_n228), .b(new_n229), .c(new_n225), .d(new_n223), .o1(\s[22] ));
  norb02aa1n03x4               g135(.a(new_n225), .b(new_n227), .out0(new_n231));
  nand23aa1n03x5               g136(.a(new_n231), .b(new_n202), .c(new_n220), .o1(new_n232));
  oai112aa1n06x5               g137(.a(new_n209), .b(new_n217), .c(new_n205), .d(new_n204), .o1(new_n233));
  tech160nm_fioai012aa1n05x5   g138(.a(new_n216), .b(new_n215), .c(new_n207), .o1(new_n234));
  nanb02aa1d24x5               g139(.a(new_n227), .b(new_n225), .out0(new_n235));
  inv000aa1d42x5               g140(.a(\a[22] ), .o1(new_n236));
  inv040aa1d32x5               g141(.a(\b[21] ), .o1(new_n237));
  oao003aa1n02x5               g142(.a(new_n236), .b(new_n237), .c(new_n226), .carry(new_n238));
  inv040aa1n06x5               g143(.a(new_n238), .o1(new_n239));
  aoai13aa1n12x5               g144(.a(new_n239), .b(new_n235), .c(new_n233), .d(new_n234), .o1(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  aoai13aa1n06x5               g146(.a(new_n241), .b(new_n232), .c(new_n188), .d(new_n192), .o1(new_n242));
  xorb03aa1n02x5               g147(.a(new_n242), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g148(.a(\b[22] ), .b(\a[23] ), .o1(new_n244));
  xorc02aa1n02x5               g149(.a(\a[23] ), .b(\b[22] ), .out0(new_n245));
  xnrc02aa1n12x5               g150(.a(\b[23] ), .b(\a[24] ), .out0(new_n246));
  aoai13aa1n03x5               g151(.a(new_n246), .b(new_n244), .c(new_n242), .d(new_n245), .o1(new_n247));
  norp02aa1n02x5               g152(.a(new_n246), .b(new_n244), .o1(new_n248));
  aob012aa1n03x5               g153(.a(new_n248), .b(new_n242), .c(new_n245), .out0(new_n249));
  nanp02aa1n03x5               g154(.a(new_n247), .b(new_n249), .o1(\s[24] ));
  norb02aa1n09x5               g155(.a(new_n245), .b(new_n246), .out0(new_n251));
  nanb03aa1n02x5               g156(.a(new_n221), .b(new_n251), .c(new_n231), .out0(new_n252));
  aoi012aa1n02x5               g157(.a(new_n248), .b(\a[24] ), .c(\b[23] ), .o1(new_n253));
  tech160nm_fiaoi012aa1n02p5x5 g158(.a(new_n253), .b(new_n240), .c(new_n251), .o1(new_n254));
  aoai13aa1n06x5               g159(.a(new_n254), .b(new_n252), .c(new_n188), .d(new_n192), .o1(new_n255));
  xorb03aa1n02x5               g160(.a(new_n255), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  tech160nm_fixorc02aa1n03p5x5 g161(.a(\a[25] ), .b(\b[24] ), .out0(new_n257));
  norp02aa1n02x5               g162(.a(\b[24] ), .b(\a[25] ), .o1(new_n258));
  tech160nm_fixnrc02aa1n05x5   g163(.a(\b[25] ), .b(\a[26] ), .out0(new_n259));
  aoai13aa1n03x5               g164(.a(new_n259), .b(new_n258), .c(new_n255), .d(new_n257), .o1(new_n260));
  oabi12aa1n02x5               g165(.a(new_n259), .b(\a[25] ), .c(\b[24] ), .out0(new_n261));
  aoai13aa1n03x5               g166(.a(new_n260), .b(new_n261), .c(new_n257), .d(new_n255), .o1(\s[26] ));
  norb02aa1n03x4               g167(.a(new_n257), .b(new_n259), .out0(new_n263));
  nano22aa1n02x5               g168(.a(new_n232), .b(new_n251), .c(new_n263), .out0(new_n264));
  aoai13aa1n06x5               g169(.a(new_n264), .b(new_n200), .c(new_n127), .d(new_n187), .o1(new_n265));
  aoai13aa1n09x5               g170(.a(new_n263), .b(new_n253), .c(new_n240), .d(new_n251), .o1(new_n266));
  aob012aa1n02x5               g171(.a(new_n261), .b(\b[25] ), .c(\a[26] ), .out0(new_n267));
  nanp03aa1n03x5               g172(.a(new_n265), .b(new_n266), .c(new_n267), .o1(new_n268));
  xorb03aa1n02x5               g173(.a(new_n268), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  inv000aa1n02x5               g174(.a(new_n264), .o1(new_n270));
  aoi012aa1n12x5               g175(.a(new_n270), .b(new_n188), .c(new_n192), .o1(new_n271));
  nand22aa1n04x5               g176(.a(new_n266), .b(new_n267), .o1(new_n272));
  norp02aa1n02x5               g177(.a(\b[26] ), .b(\a[27] ), .o1(new_n273));
  xorc02aa1n02x5               g178(.a(\a[27] ), .b(\b[26] ), .out0(new_n274));
  oaoi13aa1n03x5               g179(.a(new_n273), .b(new_n274), .c(new_n272), .d(new_n271), .o1(new_n275));
  xorc02aa1n02x5               g180(.a(\a[28] ), .b(\b[27] ), .out0(new_n276));
  oaih12aa1n02x5               g181(.a(new_n274), .b(new_n272), .c(new_n271), .o1(new_n277));
  oai112aa1n02x7               g182(.a(new_n277), .b(new_n276), .c(\b[26] ), .d(\a[27] ), .o1(new_n278));
  oaih12aa1n02x5               g183(.a(new_n278), .b(new_n275), .c(new_n276), .o1(\s[28] ));
  xorc02aa1n12x5               g184(.a(\a[29] ), .b(\b[28] ), .out0(new_n280));
  inv000aa1d42x5               g185(.a(new_n280), .o1(new_n281));
  and002aa1n02x5               g186(.a(new_n276), .b(new_n274), .o(new_n282));
  orn002aa1n02x5               g187(.a(\a[27] ), .b(\b[26] ), .o(new_n283));
  oaoi03aa1n02x5               g188(.a(\a[28] ), .b(\b[27] ), .c(new_n283), .o1(new_n284));
  aoai13aa1n02x7               g189(.a(new_n281), .b(new_n284), .c(new_n268), .d(new_n282), .o1(new_n285));
  oaih12aa1n02x5               g190(.a(new_n282), .b(new_n272), .c(new_n271), .o1(new_n286));
  nona22aa1n03x5               g191(.a(new_n286), .b(new_n284), .c(new_n281), .out0(new_n287));
  nanp02aa1n03x5               g192(.a(new_n285), .b(new_n287), .o1(\s[29] ));
  xorb03aa1n02x5               g193(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g194(.a(new_n281), .b(new_n274), .c(new_n276), .out0(new_n290));
  inv000aa1d42x5               g195(.a(\b[28] ), .o1(new_n291));
  oaib12aa1n02x5               g196(.a(new_n284), .b(new_n291), .c(\a[29] ), .out0(new_n292));
  oaib12aa1n02x5               g197(.a(new_n292), .b(\a[29] ), .c(new_n291), .out0(new_n293));
  oaoi13aa1n03x5               g198(.a(new_n293), .b(new_n290), .c(new_n272), .d(new_n271), .o1(new_n294));
  xorc02aa1n02x5               g199(.a(\a[30] ), .b(\b[29] ), .out0(new_n295));
  oaih12aa1n02x5               g200(.a(new_n290), .b(new_n272), .c(new_n271), .o1(new_n296));
  norb02aa1n02x5               g201(.a(new_n295), .b(new_n293), .out0(new_n297));
  tech160nm_finand02aa1n03p5x5 g202(.a(new_n296), .b(new_n297), .o1(new_n298));
  oaih12aa1n02x5               g203(.a(new_n298), .b(new_n294), .c(new_n295), .o1(\s[30] ));
  nano32aa1n02x4               g204(.a(new_n281), .b(new_n295), .c(new_n274), .d(new_n276), .out0(new_n300));
  aoi012aa1n02x5               g205(.a(new_n297), .b(\a[30] ), .c(\b[29] ), .o1(new_n301));
  xnrc02aa1n02x5               g206(.a(\b[30] ), .b(\a[31] ), .out0(new_n302));
  aoai13aa1n02x5               g207(.a(new_n302), .b(new_n301), .c(new_n268), .d(new_n300), .o1(new_n303));
  oaih12aa1n02x5               g208(.a(new_n300), .b(new_n272), .c(new_n271), .o1(new_n304));
  nona22aa1n03x5               g209(.a(new_n304), .b(new_n301), .c(new_n302), .out0(new_n305));
  nanp02aa1n03x5               g210(.a(new_n303), .b(new_n305), .o1(\s[31] ));
  xobna2aa1n03x5               g211(.a(new_n129), .b(new_n104), .c(new_n102), .out0(\s[3] ));
  aoi012aa1n02x5               g212(.a(new_n105), .b(new_n107), .c(new_n104), .o1(new_n308));
  xnrb03aa1n02x5               g213(.a(new_n308), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  inv000aa1d42x5               g214(.a(\a[5] ), .o1(new_n310));
  xorb03aa1n02x5               g215(.a(new_n110), .b(\b[4] ), .c(new_n310), .out0(\s[5] ));
  oaoi03aa1n02x5               g216(.a(\a[5] ), .b(\b[4] ), .c(new_n110), .o1(new_n312));
  oabi12aa1n02x5               g217(.a(new_n123), .b(new_n110), .c(new_n119), .out0(new_n313));
  oaib12aa1n02x5               g218(.a(new_n313), .b(new_n118), .c(new_n312), .out0(\s[6] ));
  xnbna2aa1n03x5               g219(.a(new_n133), .b(new_n313), .c(new_n117), .out0(\s[7] ));
  nanb03aa1n02x5               g220(.a(new_n133), .b(new_n313), .c(new_n117), .out0(new_n316));
  xnbna2aa1n03x5               g221(.a(new_n132), .b(new_n316), .c(new_n124), .out0(\s[8] ));
  xorb03aa1n02x5               g222(.a(new_n127), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


