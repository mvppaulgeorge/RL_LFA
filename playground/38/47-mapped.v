// Benchmark "adder" written by ABC on Thu Jul 18 07:52:21 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n278, new_n279, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n321, new_n322, new_n323, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n342, new_n345,
    new_n346, new_n349, new_n351, new_n352, new_n353, new_n355;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xorc02aa1n02x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  inv000aa1d42x5               g002(.a(\a[9] ), .o1(new_n98));
  nanb02aa1n02x5               g003(.a(\b[8] ), .b(new_n98), .out0(new_n99));
  nor002aa1n03x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand22aa1n03x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  nand22aa1n03x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  aoi012aa1n09x5               g007(.a(new_n100), .b(new_n101), .c(new_n102), .o1(new_n103));
  nand22aa1n12x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norp02aa1n24x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nor002aa1n12x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nand02aa1n04x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nona23aa1n09x5               g012(.a(new_n104), .b(new_n107), .c(new_n106), .d(new_n105), .out0(new_n108));
  tech160nm_fiaoi012aa1n04x5   g013(.a(new_n105), .b(new_n106), .c(new_n104), .o1(new_n109));
  oaih12aa1n12x5               g014(.a(new_n109), .b(new_n108), .c(new_n103), .o1(new_n110));
  nor002aa1n16x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nanp02aa1n04x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nand42aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nor002aa1n16x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nona23aa1n09x5               g019(.a(new_n113), .b(new_n112), .c(new_n114), .d(new_n111), .out0(new_n115));
  nand42aa1d28x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nor022aa1n16x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  nanb02aa1n09x5               g022(.a(new_n117), .b(new_n116), .out0(new_n118));
  xnrc02aa1n02x5               g023(.a(\b[4] ), .b(\a[5] ), .out0(new_n119));
  nor043aa1n06x5               g024(.a(new_n115), .b(new_n118), .c(new_n119), .o1(new_n120));
  inv000aa1d42x5               g025(.a(new_n111), .o1(new_n121));
  nanp02aa1n02x5               g026(.a(new_n114), .b(new_n112), .o1(new_n122));
  inv040aa1d32x5               g027(.a(\a[5] ), .o1(new_n123));
  inv040aa1d28x5               g028(.a(\b[4] ), .o1(new_n124));
  aoai13aa1n12x5               g029(.a(new_n116), .b(new_n117), .c(new_n124), .d(new_n123), .o1(new_n125));
  oai112aa1n06x5               g030(.a(new_n121), .b(new_n122), .c(new_n115), .d(new_n125), .o1(new_n126));
  xnrc02aa1n12x5               g031(.a(\b[8] ), .b(\a[9] ), .out0(new_n127));
  inv000aa1d42x5               g032(.a(new_n127), .o1(new_n128));
  aoai13aa1n06x5               g033(.a(new_n128), .b(new_n126), .c(new_n110), .d(new_n120), .o1(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n97), .b(new_n129), .c(new_n99), .out0(\s[10] ));
  inv040aa1d30x5               g035(.a(\a[10] ), .o1(new_n131));
  inv040aa1n09x5               g036(.a(\b[9] ), .o1(new_n132));
  nand42aa1n10x5               g037(.a(new_n132), .b(new_n131), .o1(new_n133));
  nand02aa1d28x5               g038(.a(\b[9] ), .b(\a[10] ), .o1(new_n134));
  inv000aa1d42x5               g039(.a(new_n134), .o1(new_n135));
  aoai13aa1n06x5               g040(.a(new_n133), .b(new_n135), .c(new_n129), .d(new_n99), .o1(new_n136));
  xorb03aa1n02x5               g041(.a(new_n136), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1d18x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  nanp02aa1n04x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  norb02aa1n12x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  nor042aa1n06x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand02aa1d28x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  aoai13aa1n02x7               g048(.a(new_n143), .b(new_n138), .c(new_n136), .d(new_n140), .o1(new_n144));
  aoi112aa1n02x5               g049(.a(new_n138), .b(new_n143), .c(new_n136), .d(new_n140), .o1(new_n145));
  norb02aa1n03x4               g050(.a(new_n144), .b(new_n145), .out0(\s[12] ));
  nanb02aa1n02x5               g051(.a(new_n138), .b(new_n139), .out0(new_n147));
  nano22aa1d15x5               g052(.a(new_n141), .b(new_n134), .c(new_n142), .out0(new_n148));
  nano23aa1n06x5               g053(.a(new_n147), .b(new_n127), .c(new_n148), .d(new_n133), .out0(new_n149));
  aoai13aa1n06x5               g054(.a(new_n149), .b(new_n126), .c(new_n110), .d(new_n120), .o1(new_n150));
  nanp02aa1n02x5               g055(.a(new_n138), .b(new_n142), .o1(new_n151));
  inv000aa1n02x5               g056(.a(new_n151), .o1(new_n152));
  oai112aa1n06x5               g057(.a(new_n133), .b(new_n134), .c(\b[8] ), .d(\a[9] ), .o1(new_n153));
  nanp03aa1n06x5               g058(.a(new_n148), .b(new_n153), .c(new_n140), .o1(new_n154));
  nona22aa1n12x5               g059(.a(new_n154), .b(new_n152), .c(new_n141), .out0(new_n155));
  inv000aa1d42x5               g060(.a(new_n155), .o1(new_n156));
  tech160nm_fixnrc02aa1n04x5   g061(.a(\b[12] ), .b(\a[13] ), .out0(new_n157));
  xobna2aa1n03x5               g062(.a(new_n157), .b(new_n150), .c(new_n156), .out0(\s[13] ));
  inv040aa1d32x5               g063(.a(\a[13] ), .o1(new_n159));
  inv040aa1d28x5               g064(.a(\b[12] ), .o1(new_n160));
  nand02aa1d24x5               g065(.a(new_n160), .b(new_n159), .o1(new_n161));
  aoai13aa1n02x5               g066(.a(new_n161), .b(new_n157), .c(new_n150), .d(new_n156), .o1(new_n162));
  xorb03aa1n02x5               g067(.a(new_n162), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  and002aa1n03x5               g068(.a(\b[12] ), .b(\a[13] ), .o(new_n164));
  nor042aa1n02x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  nand42aa1n03x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  nona23aa1n09x5               g071(.a(new_n166), .b(new_n161), .c(new_n164), .d(new_n165), .out0(new_n167));
  oaoi03aa1n12x5               g072(.a(\a[14] ), .b(\b[13] ), .c(new_n161), .o1(new_n168));
  inv000aa1d42x5               g073(.a(new_n168), .o1(new_n169));
  aoai13aa1n06x5               g074(.a(new_n169), .b(new_n167), .c(new_n150), .d(new_n156), .o1(new_n170));
  xorb03aa1n02x5               g075(.a(new_n170), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nand02aa1d24x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  nor042aa1n04x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  nor042aa1n04x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  nanp02aa1n09x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  nanb02aa1n02x5               g080(.a(new_n174), .b(new_n175), .out0(new_n176));
  inv000aa1d42x5               g081(.a(new_n176), .o1(new_n177));
  aoi112aa1n03x5               g082(.a(new_n177), .b(new_n173), .c(new_n170), .d(new_n172), .o1(new_n178));
  aoai13aa1n03x5               g083(.a(new_n177), .b(new_n173), .c(new_n170), .d(new_n172), .o1(new_n179));
  norb02aa1n03x4               g084(.a(new_n179), .b(new_n178), .out0(\s[16] ));
  aoi012aa1n12x5               g085(.a(new_n126), .b(new_n110), .c(new_n120), .o1(new_n181));
  nano23aa1n09x5               g086(.a(new_n174), .b(new_n173), .c(new_n175), .d(new_n172), .out0(new_n182));
  norb02aa1n03x5               g087(.a(new_n182), .b(new_n167), .out0(new_n183));
  nanp02aa1n04x5               g088(.a(new_n149), .b(new_n183), .o1(new_n184));
  aoi112aa1n02x5               g089(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n185));
  nand22aa1n03x5               g090(.a(new_n182), .b(new_n168), .o1(new_n186));
  nona22aa1n02x4               g091(.a(new_n186), .b(new_n185), .c(new_n174), .out0(new_n187));
  aoi012aa1d24x5               g092(.a(new_n187), .b(new_n155), .c(new_n183), .o1(new_n188));
  oai012aa1d24x5               g093(.a(new_n188), .b(new_n181), .c(new_n184), .o1(new_n189));
  xorb03aa1n02x5               g094(.a(new_n189), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nor002aa1d32x5               g095(.a(\b[17] ), .b(\a[18] ), .o1(new_n191));
  nand42aa1d28x5               g096(.a(\b[17] ), .b(\a[18] ), .o1(new_n192));
  nanb02aa1n09x5               g097(.a(new_n191), .b(new_n192), .out0(new_n193));
  inv040aa1d28x5               g098(.a(\a[17] ), .o1(new_n194));
  inv040aa1d32x5               g099(.a(\b[16] ), .o1(new_n195));
  tech160nm_fioaoi03aa1n03p5x5 g100(.a(new_n194), .b(new_n195), .c(new_n189), .o1(new_n196));
  tech160nm_fixorc02aa1n02p5x5 g101(.a(new_n196), .b(new_n193), .out0(\s[18] ));
  nona23aa1n09x5               g102(.a(new_n148), .b(new_n133), .c(new_n127), .d(new_n147), .out0(new_n198));
  nona23aa1n03x5               g103(.a(new_n182), .b(new_n166), .c(new_n157), .d(new_n165), .out0(new_n199));
  nor042aa1n02x5               g104(.a(new_n199), .b(new_n198), .o1(new_n200));
  aoai13aa1n06x5               g105(.a(new_n200), .b(new_n126), .c(new_n110), .d(new_n120), .o1(new_n201));
  nanp02aa1n02x5               g106(.a(new_n195), .b(new_n194), .o1(new_n202));
  nanp02aa1n02x5               g107(.a(\b[16] ), .b(\a[17] ), .o1(new_n203));
  nano22aa1d15x5               g108(.a(new_n193), .b(new_n202), .c(new_n203), .out0(new_n204));
  inv000aa1d42x5               g109(.a(new_n204), .o1(new_n205));
  aoai13aa1n12x5               g110(.a(new_n192), .b(new_n191), .c(new_n194), .d(new_n195), .o1(new_n206));
  aoai13aa1n02x7               g111(.a(new_n206), .b(new_n205), .c(new_n201), .d(new_n188), .o1(new_n207));
  xorb03aa1n02x5               g112(.a(new_n207), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g113(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  nanp02aa1n04x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  nanb02aa1n12x5               g116(.a(new_n210), .b(new_n211), .out0(new_n212));
  inv000aa1d42x5               g117(.a(new_n212), .o1(new_n213));
  inv000aa1d42x5               g118(.a(\b[19] ), .o1(new_n214));
  nanb02aa1n12x5               g119(.a(\a[20] ), .b(new_n214), .out0(new_n215));
  nand02aa1n04x5               g120(.a(\b[19] ), .b(\a[20] ), .o1(new_n216));
  nanp02aa1n09x5               g121(.a(new_n215), .b(new_n216), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  aoi112aa1n03x4               g123(.a(new_n210), .b(new_n218), .c(new_n207), .d(new_n213), .o1(new_n219));
  inv000aa1d42x5               g124(.a(new_n210), .o1(new_n220));
  inv000aa1d42x5               g125(.a(new_n206), .o1(new_n221));
  aoai13aa1n06x5               g126(.a(new_n213), .b(new_n221), .c(new_n189), .d(new_n204), .o1(new_n222));
  aoi012aa1n03x5               g127(.a(new_n217), .b(new_n222), .c(new_n220), .o1(new_n223));
  norp02aa1n03x5               g128(.a(new_n223), .b(new_n219), .o1(\s[20] ));
  nona22aa1d24x5               g129(.a(new_n204), .b(new_n212), .c(new_n217), .out0(new_n225));
  nanp02aa1n02x5               g130(.a(new_n210), .b(new_n216), .o1(new_n226));
  nor043aa1n03x5               g131(.a(new_n206), .b(new_n212), .c(new_n217), .o1(new_n227));
  nano22aa1n03x7               g132(.a(new_n227), .b(new_n215), .c(new_n226), .out0(new_n228));
  aoai13aa1n06x5               g133(.a(new_n228), .b(new_n225), .c(new_n201), .d(new_n188), .o1(new_n229));
  xorb03aa1n02x5               g134(.a(new_n229), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1d18x5               g135(.a(\b[20] ), .b(\a[21] ), .o1(new_n231));
  xnrc02aa1n12x5               g136(.a(\b[20] ), .b(\a[21] ), .out0(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  xnrc02aa1n12x5               g138(.a(\b[21] ), .b(\a[22] ), .out0(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  aoi112aa1n02x5               g140(.a(new_n231), .b(new_n235), .c(new_n229), .d(new_n233), .o1(new_n236));
  inv020aa1n06x5               g141(.a(new_n231), .o1(new_n237));
  inv000aa1d42x5               g142(.a(new_n225), .o1(new_n238));
  nor002aa1n02x5               g143(.a(\b[19] ), .b(\a[20] ), .o1(new_n239));
  nona23aa1n06x5               g144(.a(new_n211), .b(new_n216), .c(new_n239), .d(new_n210), .out0(new_n240));
  oai112aa1n06x5               g145(.a(new_n226), .b(new_n215), .c(new_n240), .d(new_n206), .o1(new_n241));
  aoai13aa1n06x5               g146(.a(new_n233), .b(new_n241), .c(new_n189), .d(new_n238), .o1(new_n242));
  aoi012aa1n03x5               g147(.a(new_n234), .b(new_n242), .c(new_n237), .o1(new_n243));
  nor002aa1n02x5               g148(.a(new_n243), .b(new_n236), .o1(\s[22] ));
  inv040aa1n02x5               g149(.a(new_n240), .o1(new_n245));
  nor042aa1n06x5               g150(.a(new_n234), .b(new_n232), .o1(new_n246));
  nand23aa1d12x5               g151(.a(new_n245), .b(new_n246), .c(new_n204), .o1(new_n247));
  oao003aa1n03x5               g152(.a(\a[22] ), .b(\b[21] ), .c(new_n237), .carry(new_n248));
  aobi12aa1n12x5               g153(.a(new_n248), .b(new_n241), .c(new_n246), .out0(new_n249));
  aoai13aa1n02x7               g154(.a(new_n249), .b(new_n247), .c(new_n201), .d(new_n188), .o1(new_n250));
  xorb03aa1n02x5               g155(.a(new_n250), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor002aa1d32x5               g156(.a(\b[22] ), .b(\a[23] ), .o1(new_n252));
  xorc02aa1n12x5               g157(.a(\a[23] ), .b(\b[22] ), .out0(new_n253));
  xorc02aa1n12x5               g158(.a(\a[24] ), .b(\b[23] ), .out0(new_n254));
  aoi112aa1n03x4               g159(.a(new_n252), .b(new_n254), .c(new_n250), .d(new_n253), .o1(new_n255));
  inv000aa1d42x5               g160(.a(new_n252), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n247), .o1(new_n257));
  inv000aa1n02x5               g162(.a(new_n249), .o1(new_n258));
  aoai13aa1n06x5               g163(.a(new_n253), .b(new_n258), .c(new_n189), .d(new_n257), .o1(new_n259));
  aobi12aa1n03x5               g164(.a(new_n254), .b(new_n259), .c(new_n256), .out0(new_n260));
  norp02aa1n03x5               g165(.a(new_n260), .b(new_n255), .o1(\s[24] ));
  nano32aa1n03x7               g166(.a(new_n225), .b(new_n254), .c(new_n246), .d(new_n253), .out0(new_n262));
  inv000aa1n02x5               g167(.a(new_n262), .o1(new_n263));
  xnrc02aa1n02x5               g168(.a(\b[22] ), .b(\a[23] ), .out0(new_n264));
  norb02aa1n09x5               g169(.a(new_n254), .b(new_n264), .out0(new_n265));
  nanp02aa1n02x5               g170(.a(new_n254), .b(new_n253), .o1(new_n266));
  oao003aa1n03x5               g171(.a(\a[24] ), .b(\b[23] ), .c(new_n256), .carry(new_n267));
  oai012aa1n02x5               g172(.a(new_n267), .b(new_n266), .c(new_n248), .o1(new_n268));
  aoi013aa1n03x5               g173(.a(new_n268), .b(new_n241), .c(new_n246), .d(new_n265), .o1(new_n269));
  aoai13aa1n03x5               g174(.a(new_n269), .b(new_n263), .c(new_n201), .d(new_n188), .o1(new_n270));
  xorb03aa1n02x5               g175(.a(new_n270), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g176(.a(\b[24] ), .b(\a[25] ), .o1(new_n272));
  xorc02aa1n12x5               g177(.a(\a[25] ), .b(\b[24] ), .out0(new_n273));
  tech160nm_fixorc02aa1n03p5x5 g178(.a(\a[26] ), .b(\b[25] ), .out0(new_n274));
  aoi112aa1n02x5               g179(.a(new_n272), .b(new_n274), .c(new_n270), .d(new_n273), .o1(new_n275));
  inv000aa1n02x5               g180(.a(new_n272), .o1(new_n276));
  inv040aa1n03x5               g181(.a(new_n269), .o1(new_n277));
  aoai13aa1n06x5               g182(.a(new_n273), .b(new_n277), .c(new_n189), .d(new_n262), .o1(new_n278));
  aobi12aa1n03x5               g183(.a(new_n274), .b(new_n278), .c(new_n276), .out0(new_n279));
  nor002aa1n02x5               g184(.a(new_n279), .b(new_n275), .o1(\s[26] ));
  inv030aa1n02x5               g185(.a(new_n103), .o1(new_n281));
  nano23aa1d15x5               g186(.a(new_n106), .b(new_n105), .c(new_n107), .d(new_n104), .out0(new_n282));
  aobi12aa1n06x5               g187(.a(new_n109), .b(new_n282), .c(new_n281), .out0(new_n283));
  nanb02aa1n06x5               g188(.a(new_n111), .b(new_n112), .out0(new_n284));
  norb02aa1n06x4               g189(.a(new_n113), .b(new_n114), .out0(new_n285));
  inv000aa1d42x5               g190(.a(new_n118), .o1(new_n286));
  nona23aa1n03x5               g191(.a(new_n286), .b(new_n285), .c(new_n119), .d(new_n284), .out0(new_n287));
  norb03aa1n03x4               g192(.a(new_n285), .b(new_n125), .c(new_n284), .out0(new_n288));
  nano22aa1n02x4               g193(.a(new_n288), .b(new_n121), .c(new_n122), .out0(new_n289));
  oai012aa1n03x5               g194(.a(new_n289), .b(new_n287), .c(new_n283), .o1(new_n290));
  aoi013aa1n02x4               g195(.a(new_n141), .b(new_n148), .c(new_n153), .d(new_n140), .o1(new_n291));
  aoi112aa1n02x7               g196(.a(new_n185), .b(new_n174), .c(new_n182), .d(new_n168), .o1(new_n292));
  aoai13aa1n03x5               g197(.a(new_n292), .b(new_n199), .c(new_n291), .d(new_n151), .o1(new_n293));
  and002aa1n09x5               g198(.a(new_n274), .b(new_n273), .o(new_n294));
  nano22aa1d15x5               g199(.a(new_n247), .b(new_n294), .c(new_n265), .out0(new_n295));
  aoai13aa1n06x5               g200(.a(new_n295), .b(new_n293), .c(new_n290), .d(new_n200), .o1(new_n296));
  nano22aa1n03x7               g201(.a(new_n228), .b(new_n246), .c(new_n265), .out0(new_n297));
  oao003aa1n02x5               g202(.a(\a[26] ), .b(\b[25] ), .c(new_n276), .carry(new_n298));
  inv000aa1d42x5               g203(.a(new_n298), .o1(new_n299));
  oaoi13aa1n09x5               g204(.a(new_n299), .b(new_n294), .c(new_n297), .d(new_n268), .o1(new_n300));
  xorc02aa1n12x5               g205(.a(\a[27] ), .b(\b[26] ), .out0(new_n301));
  xnbna2aa1n03x5               g206(.a(new_n301), .b(new_n300), .c(new_n296), .out0(\s[27] ));
  norp02aa1n02x5               g207(.a(\b[26] ), .b(\a[27] ), .o1(new_n303));
  inv040aa1n03x5               g208(.a(new_n303), .o1(new_n304));
  inv000aa1d42x5               g209(.a(new_n301), .o1(new_n305));
  aoi012aa1n02x7               g210(.a(new_n305), .b(new_n300), .c(new_n296), .o1(new_n306));
  xnrc02aa1n12x5               g211(.a(\b[27] ), .b(\a[28] ), .out0(new_n307));
  nano22aa1n02x4               g212(.a(new_n306), .b(new_n304), .c(new_n307), .out0(new_n308));
  nona32aa1n09x5               g213(.a(new_n241), .b(new_n266), .c(new_n234), .d(new_n232), .out0(new_n309));
  inv000aa1n02x5               g214(.a(new_n268), .o1(new_n310));
  inv000aa1d42x5               g215(.a(new_n294), .o1(new_n311));
  aoai13aa1n12x5               g216(.a(new_n298), .b(new_n311), .c(new_n309), .d(new_n310), .o1(new_n312));
  aoai13aa1n06x5               g217(.a(new_n301), .b(new_n312), .c(new_n189), .d(new_n295), .o1(new_n313));
  aoi012aa1n03x5               g218(.a(new_n307), .b(new_n313), .c(new_n304), .o1(new_n314));
  norp02aa1n03x5               g219(.a(new_n314), .b(new_n308), .o1(\s[28] ));
  norb02aa1n02x5               g220(.a(new_n301), .b(new_n307), .out0(new_n316));
  aoai13aa1n06x5               g221(.a(new_n316), .b(new_n312), .c(new_n189), .d(new_n295), .o1(new_n317));
  oao003aa1n02x5               g222(.a(\a[28] ), .b(\b[27] ), .c(new_n304), .carry(new_n318));
  xnrc02aa1n02x5               g223(.a(\b[28] ), .b(\a[29] ), .out0(new_n319));
  aoi012aa1n03x5               g224(.a(new_n319), .b(new_n317), .c(new_n318), .o1(new_n320));
  inv000aa1n02x5               g225(.a(new_n316), .o1(new_n321));
  tech160nm_fiaoi012aa1n02p5x5 g226(.a(new_n321), .b(new_n300), .c(new_n296), .o1(new_n322));
  nano22aa1n02x4               g227(.a(new_n322), .b(new_n318), .c(new_n319), .out0(new_n323));
  nor002aa1n02x5               g228(.a(new_n320), .b(new_n323), .o1(\s[29] ));
  xorb03aa1n02x5               g229(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n12x5               g230(.a(new_n301), .b(new_n319), .c(new_n307), .out0(new_n326));
  aoai13aa1n06x5               g231(.a(new_n326), .b(new_n312), .c(new_n189), .d(new_n295), .o1(new_n327));
  oao003aa1n02x5               g232(.a(\a[29] ), .b(\b[28] ), .c(new_n318), .carry(new_n328));
  xnrc02aa1n02x5               g233(.a(\b[29] ), .b(\a[30] ), .out0(new_n329));
  aoi012aa1n03x5               g234(.a(new_n329), .b(new_n327), .c(new_n328), .o1(new_n330));
  inv000aa1d42x5               g235(.a(new_n326), .o1(new_n331));
  aoi012aa1n02x5               g236(.a(new_n331), .b(new_n300), .c(new_n296), .o1(new_n332));
  nano22aa1n02x4               g237(.a(new_n332), .b(new_n328), .c(new_n329), .out0(new_n333));
  norp02aa1n03x5               g238(.a(new_n330), .b(new_n333), .o1(\s[30] ));
  norb02aa1n03x4               g239(.a(new_n326), .b(new_n329), .out0(new_n335));
  inv020aa1n03x5               g240(.a(new_n335), .o1(new_n336));
  aoi012aa1n02x7               g241(.a(new_n336), .b(new_n300), .c(new_n296), .o1(new_n337));
  oao003aa1n02x5               g242(.a(\a[30] ), .b(\b[29] ), .c(new_n328), .carry(new_n338));
  xnrc02aa1n02x5               g243(.a(\b[30] ), .b(\a[31] ), .out0(new_n339));
  nano22aa1n02x4               g244(.a(new_n337), .b(new_n338), .c(new_n339), .out0(new_n340));
  aoai13aa1n06x5               g245(.a(new_n335), .b(new_n312), .c(new_n189), .d(new_n295), .o1(new_n341));
  aoi012aa1n03x5               g246(.a(new_n339), .b(new_n341), .c(new_n338), .o1(new_n342));
  norp02aa1n03x5               g247(.a(new_n342), .b(new_n340), .o1(\s[31] ));
  xnrb03aa1n02x5               g248(.a(new_n103), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  nona22aa1n02x4               g249(.a(new_n107), .b(new_n103), .c(new_n106), .out0(new_n345));
  aoib12aa1n02x5               g250(.a(new_n106), .b(new_n104), .c(new_n105), .out0(new_n346));
  aboi22aa1n03x5               g251(.a(new_n105), .b(new_n110), .c(new_n345), .d(new_n346), .out0(\s[4] ));
  xorb03aa1n02x5               g252(.a(new_n110), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  tech160nm_fioaoi03aa1n03p5x5 g253(.a(\a[5] ), .b(\b[4] ), .c(new_n283), .o1(new_n349));
  xorb03aa1n02x5               g254(.a(new_n349), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  inv000aa1d42x5               g255(.a(new_n125), .o1(new_n351));
  aoai13aa1n06x5               g256(.a(new_n285), .b(new_n351), .c(new_n349), .d(new_n286), .o1(new_n352));
  aoi112aa1n02x5               g257(.a(new_n351), .b(new_n285), .c(new_n349), .d(new_n286), .o1(new_n353));
  norb02aa1n02x5               g258(.a(new_n352), .b(new_n353), .out0(\s[7] ));
  inv000aa1d42x5               g259(.a(new_n114), .o1(new_n355));
  xobna2aa1n03x5               g260(.a(new_n284), .b(new_n352), .c(new_n355), .out0(\s[8] ));
  xorb03aa1n02x5               g261(.a(new_n181), .b(\b[8] ), .c(new_n98), .out0(\s[9] ));
endmodule

