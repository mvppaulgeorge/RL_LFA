// Benchmark "adder" written by ABC on Thu Jul 18 05:44:20 2024

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
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n169, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n209, new_n210,
    new_n211, new_n212, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n253, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n313, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n320, new_n321, new_n324,
    new_n325, new_n327, new_n328, new_n330, new_n331, new_n332, new_n334,
    new_n335, new_n336, new_n337, new_n339, new_n340, new_n341;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nand02aa1n20x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nor042aa1n06x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  nanb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  nand02aa1n20x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(\a[2] ), .o1(new_n101));
  inv000aa1d42x5               g006(.a(\b[1] ), .o1(new_n102));
  nand22aa1n12x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  oaoi03aa1n12x5               g008(.a(new_n101), .b(new_n102), .c(new_n103), .o1(new_n104));
  nor002aa1d32x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nand02aa1n06x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nor022aa1n16x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  tech160nm_finand02aa1n03p5x5 g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nona23aa1n12x5               g013(.a(new_n108), .b(new_n106), .c(new_n105), .d(new_n107), .out0(new_n109));
  inv000aa1d42x5               g014(.a(\a[3] ), .o1(new_n110));
  inv000aa1d42x5               g015(.a(\b[2] ), .o1(new_n111));
  aoai13aa1n04x5               g016(.a(new_n106), .b(new_n105), .c(new_n110), .d(new_n111), .o1(new_n112));
  oai012aa1n12x5               g017(.a(new_n112), .b(new_n109), .c(new_n104), .o1(new_n113));
  xnrc02aa1n12x5               g018(.a(\b[5] ), .b(\a[6] ), .out0(new_n114));
  xnrc02aa1n12x5               g019(.a(\b[4] ), .b(\a[5] ), .out0(new_n115));
  nand42aa1n08x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  nor022aa1n16x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nor002aa1d32x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nanp02aa1n09x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nona23aa1d18x5               g024(.a(new_n116), .b(new_n119), .c(new_n118), .d(new_n117), .out0(new_n120));
  nor003aa1d16x5               g025(.a(new_n120), .b(new_n115), .c(new_n114), .o1(new_n121));
  inv040aa1d32x5               g026(.a(\a[6] ), .o1(new_n122));
  inv000aa1d42x5               g027(.a(\b[5] ), .o1(new_n123));
  nor002aa1d24x5               g028(.a(\b[4] ), .b(\a[5] ), .o1(new_n124));
  tech160nm_fioaoi03aa1n02p5x5 g029(.a(new_n122), .b(new_n123), .c(new_n124), .o1(new_n125));
  norb03aa1n03x5               g030(.a(new_n116), .b(new_n118), .c(new_n117), .out0(new_n126));
  obai22aa1n09x5               g031(.a(new_n116), .b(new_n126), .c(new_n120), .d(new_n125), .out0(new_n127));
  aoi012aa1n06x5               g032(.a(new_n127), .b(new_n113), .c(new_n121), .o1(new_n128));
  tech160nm_fioai012aa1n04x5   g033(.a(new_n128), .b(\b[8] ), .c(\a[9] ), .o1(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n99), .b(new_n129), .c(new_n100), .out0(\s[10] ));
  nano22aa1n02x4               g035(.a(new_n98), .b(new_n97), .c(new_n100), .out0(new_n131));
  nor042aa1n06x5               g036(.a(\b[8] ), .b(\a[9] ), .o1(new_n132));
  tech160nm_fiao0012aa1n02p5x5 g037(.a(new_n98), .b(new_n132), .c(new_n97), .o(new_n133));
  nor002aa1d32x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand42aa1d28x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  aoai13aa1n04x5               g041(.a(new_n136), .b(new_n133), .c(new_n129), .d(new_n131), .o1(new_n137));
  aoi112aa1n02x5               g042(.a(new_n136), .b(new_n133), .c(new_n129), .d(new_n131), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n137), .b(new_n138), .out0(\s[11] ));
  inv000aa1d42x5               g044(.a(new_n134), .o1(new_n140));
  nor042aa1n06x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand42aa1d28x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  norp02aa1n02x5               g048(.a(new_n141), .b(new_n134), .o1(new_n144));
  nand03aa1n02x5               g049(.a(new_n137), .b(new_n142), .c(new_n144), .o1(new_n145));
  aoai13aa1n03x5               g050(.a(new_n145), .b(new_n143), .c(new_n140), .d(new_n137), .o1(\s[12] ));
  nano23aa1n09x5               g051(.a(new_n134), .b(new_n141), .c(new_n142), .d(new_n135), .out0(new_n147));
  nano23aa1d15x5               g052(.a(new_n132), .b(new_n98), .c(new_n97), .d(new_n100), .out0(new_n148));
  nand22aa1n09x5               g053(.a(new_n148), .b(new_n147), .o1(new_n149));
  inv000aa1d42x5               g054(.a(new_n149), .o1(new_n150));
  aoai13aa1n06x5               g055(.a(new_n150), .b(new_n127), .c(new_n113), .d(new_n121), .o1(new_n151));
  aoai13aa1n03x5               g056(.a(new_n135), .b(new_n98), .c(new_n132), .d(new_n97), .o1(new_n152));
  nand42aa1n02x5               g057(.a(new_n152), .b(new_n144), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(new_n153), .b(new_n142), .o1(new_n154));
  nor002aa1d32x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nand02aa1d06x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  norb02aa1n02x5               g061(.a(new_n156), .b(new_n155), .out0(new_n157));
  xnbna2aa1n03x5               g062(.a(new_n157), .b(new_n151), .c(new_n154), .out0(\s[13] ));
  nand22aa1n03x5               g063(.a(new_n151), .b(new_n154), .o1(new_n159));
  inv030aa1n02x5               g064(.a(new_n155), .o1(new_n160));
  nor002aa1n04x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  tech160nm_finand02aa1n05x5   g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  oaib12aa1n02x5               g067(.a(new_n160), .b(new_n161), .c(new_n162), .out0(new_n163));
  aoi012aa1n02x5               g068(.a(new_n163), .b(new_n159), .c(new_n157), .o1(new_n164));
  nano23aa1n06x5               g069(.a(new_n155), .b(new_n161), .c(new_n162), .d(new_n156), .out0(new_n165));
  nanp02aa1n03x5               g070(.a(new_n159), .b(new_n165), .o1(new_n166));
  tech160nm_fioaoi03aa1n03p5x5 g071(.a(\a[14] ), .b(\b[13] ), .c(new_n160), .o1(new_n167));
  inv000aa1d42x5               g072(.a(new_n167), .o1(new_n168));
  aoi012aa1n02x5               g073(.a(new_n161), .b(new_n166), .c(new_n168), .o1(new_n169));
  norp02aa1n02x5               g074(.a(new_n169), .b(new_n164), .o1(\s[14] ));
  nor002aa1n12x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nand42aa1n10x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  xnbna2aa1n03x5               g078(.a(new_n173), .b(new_n166), .c(new_n168), .out0(\s[15] ));
  inv000aa1d42x5               g079(.a(new_n171), .o1(new_n175));
  aoai13aa1n03x5               g080(.a(new_n173), .b(new_n167), .c(new_n159), .d(new_n165), .o1(new_n176));
  nor042aa1n04x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  nand02aa1d28x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  norb02aa1n02x5               g083(.a(new_n178), .b(new_n177), .out0(new_n179));
  inv000aa1d42x5               g084(.a(new_n178), .o1(new_n180));
  oai022aa1n02x5               g085(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n181));
  nona22aa1n02x5               g086(.a(new_n176), .b(new_n180), .c(new_n181), .out0(new_n182));
  aoai13aa1n02x5               g087(.a(new_n182), .b(new_n179), .c(new_n176), .d(new_n175), .o1(\s[16] ));
  nano23aa1n03x7               g088(.a(new_n171), .b(new_n177), .c(new_n178), .d(new_n172), .out0(new_n184));
  nano22aa1n09x5               g089(.a(new_n149), .b(new_n165), .c(new_n184), .out0(new_n185));
  aoai13aa1n12x5               g090(.a(new_n185), .b(new_n127), .c(new_n113), .d(new_n121), .o1(new_n186));
  nano22aa1n03x5               g091(.a(new_n177), .b(new_n172), .c(new_n178), .out0(new_n187));
  oai012aa1n02x7               g092(.a(new_n156), .b(\b[13] ), .c(\a[14] ), .o1(new_n188));
  tech160nm_fioai012aa1n03p5x5 g093(.a(new_n162), .b(\b[14] ), .c(\a[15] ), .o1(new_n189));
  nano23aa1n03x7               g094(.a(new_n189), .b(new_n188), .c(new_n160), .d(new_n142), .out0(new_n190));
  oab012aa1n02x5               g095(.a(new_n189), .b(new_n155), .c(new_n161), .out0(new_n191));
  ao0022aa1n03x7               g096(.a(new_n191), .b(new_n187), .c(new_n181), .d(new_n178), .o(new_n192));
  aoi013aa1n06x4               g097(.a(new_n192), .b(new_n153), .c(new_n187), .d(new_n190), .o1(new_n193));
  nanp02aa1n09x5               g098(.a(new_n186), .b(new_n193), .o1(new_n194));
  xorc02aa1n02x5               g099(.a(\a[17] ), .b(\b[16] ), .out0(new_n195));
  aoai13aa1n02x5               g100(.a(new_n184), .b(new_n167), .c(new_n159), .d(new_n165), .o1(new_n196));
  aoi012aa1n02x5               g101(.a(new_n195), .b(new_n178), .c(new_n181), .o1(new_n197));
  aoi022aa1n02x5               g102(.a(new_n196), .b(new_n197), .c(new_n195), .d(new_n194), .o1(\s[17] ));
  norp02aa1n02x5               g103(.a(\b[17] ), .b(\a[18] ), .o1(new_n199));
  nand02aa1n03x5               g104(.a(\b[17] ), .b(\a[18] ), .o1(new_n200));
  obai22aa1n02x7               g105(.a(new_n200), .b(new_n199), .c(\a[17] ), .d(\b[16] ), .out0(new_n201));
  aoi012aa1n02x5               g106(.a(new_n201), .b(new_n194), .c(new_n195), .o1(new_n202));
  inv040aa1d30x5               g107(.a(\a[17] ), .o1(new_n203));
  inv040aa1d32x5               g108(.a(\a[18] ), .o1(new_n204));
  xroi22aa1d06x4               g109(.a(new_n203), .b(\b[16] ), .c(new_n204), .d(\b[17] ), .out0(new_n205));
  oai022aa1n12x5               g110(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n206));
  aoi022aa1n02x5               g111(.a(new_n194), .b(new_n205), .c(new_n200), .d(new_n206), .o1(new_n207));
  oab012aa1n03x5               g112(.a(new_n202), .b(new_n207), .c(new_n199), .out0(\s[18] ));
  and002aa1n02x5               g113(.a(new_n206), .b(new_n200), .o(new_n209));
  xorc02aa1n02x5               g114(.a(\a[19] ), .b(\b[18] ), .out0(new_n210));
  aoai13aa1n06x5               g115(.a(new_n210), .b(new_n209), .c(new_n194), .d(new_n205), .o1(new_n211));
  aoi112aa1n02x5               g116(.a(new_n210), .b(new_n209), .c(new_n194), .d(new_n205), .o1(new_n212));
  norb02aa1n03x4               g117(.a(new_n211), .b(new_n212), .out0(\s[19] ));
  xnrc02aa1n02x5               g118(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g119(.a(\a[19] ), .o1(new_n215));
  inv000aa1d42x5               g120(.a(\b[18] ), .o1(new_n216));
  nanp02aa1n02x5               g121(.a(new_n216), .b(new_n215), .o1(new_n217));
  xorc02aa1n02x5               g122(.a(\a[20] ), .b(\b[19] ), .out0(new_n218));
  nand02aa1d16x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  inv000aa1d42x5               g124(.a(new_n219), .o1(new_n220));
  oai022aa1n02x5               g125(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n221));
  nona22aa1n02x5               g126(.a(new_n211), .b(new_n220), .c(new_n221), .out0(new_n222));
  aoai13aa1n03x5               g127(.a(new_n222), .b(new_n218), .c(new_n217), .d(new_n211), .o1(\s[20] ));
  inv030aa1d32x5               g128(.a(\a[20] ), .o1(new_n224));
  xroi22aa1d06x4               g129(.a(new_n215), .b(\b[18] ), .c(new_n224), .d(\b[19] ), .out0(new_n225));
  nand22aa1n12x5               g130(.a(new_n225), .b(new_n205), .o1(new_n226));
  inv000aa1d42x5               g131(.a(new_n226), .o1(new_n227));
  nanb02aa1n03x5               g132(.a(\b[19] ), .b(new_n224), .out0(new_n228));
  oai112aa1n04x5               g133(.a(new_n228), .b(new_n219), .c(new_n216), .d(new_n215), .o1(new_n229));
  nand03aa1n02x5               g134(.a(new_n206), .b(new_n217), .c(new_n200), .o1(new_n230));
  nanp02aa1n02x5               g135(.a(new_n221), .b(new_n219), .o1(new_n231));
  oai012aa1n06x5               g136(.a(new_n231), .b(new_n230), .c(new_n229), .o1(new_n232));
  xorc02aa1n02x5               g137(.a(\a[21] ), .b(\b[20] ), .out0(new_n233));
  aoai13aa1n06x5               g138(.a(new_n233), .b(new_n232), .c(new_n194), .d(new_n227), .o1(new_n234));
  aoi112aa1n02x5               g139(.a(new_n233), .b(new_n232), .c(new_n194), .d(new_n227), .o1(new_n235));
  norb02aa1n03x4               g140(.a(new_n234), .b(new_n235), .out0(\s[21] ));
  inv000aa1d42x5               g141(.a(\a[21] ), .o1(new_n237));
  nanb02aa1n02x5               g142(.a(\b[20] ), .b(new_n237), .out0(new_n238));
  xorc02aa1n02x5               g143(.a(\a[22] ), .b(\b[21] ), .out0(new_n239));
  and002aa1n02x5               g144(.a(\b[21] ), .b(\a[22] ), .o(new_n240));
  oai022aa1n02x5               g145(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n241));
  nona22aa1n02x5               g146(.a(new_n234), .b(new_n240), .c(new_n241), .out0(new_n242));
  aoai13aa1n03x5               g147(.a(new_n242), .b(new_n239), .c(new_n238), .d(new_n234), .o1(\s[22] ));
  inv040aa1d32x5               g148(.a(\a[22] ), .o1(new_n244));
  xroi22aa1d04x5               g149(.a(new_n237), .b(\b[20] ), .c(new_n244), .d(\b[21] ), .out0(new_n245));
  and003aa1n02x5               g150(.a(new_n205), .b(new_n245), .c(new_n225), .o(new_n246));
  oaoi03aa1n02x5               g151(.a(\a[22] ), .b(\b[21] ), .c(new_n238), .o1(new_n247));
  tech160nm_fiao0012aa1n02p5x5 g152(.a(new_n247), .b(new_n232), .c(new_n245), .o(new_n248));
  xorc02aa1n02x5               g153(.a(\a[23] ), .b(\b[22] ), .out0(new_n249));
  aoai13aa1n06x5               g154(.a(new_n249), .b(new_n248), .c(new_n194), .d(new_n246), .o1(new_n250));
  aoi112aa1n02x5               g155(.a(new_n249), .b(new_n248), .c(new_n194), .d(new_n246), .o1(new_n251));
  norb02aa1n03x4               g156(.a(new_n250), .b(new_n251), .out0(\s[23] ));
  norp02aa1n02x5               g157(.a(\b[22] ), .b(\a[23] ), .o1(new_n253));
  inv000aa1d42x5               g158(.a(new_n253), .o1(new_n254));
  xorc02aa1n02x5               g159(.a(\a[24] ), .b(\b[23] ), .out0(new_n255));
  and002aa1n02x5               g160(.a(\b[23] ), .b(\a[24] ), .o(new_n256));
  oai022aa1n02x5               g161(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n257));
  nona22aa1n02x5               g162(.a(new_n250), .b(new_n256), .c(new_n257), .out0(new_n258));
  aoai13aa1n03x5               g163(.a(new_n258), .b(new_n255), .c(new_n254), .d(new_n250), .o1(\s[24] ));
  inv000aa1d42x5               g164(.a(\a[23] ), .o1(new_n260));
  inv000aa1d42x5               g165(.a(\a[24] ), .o1(new_n261));
  xroi22aa1d04x5               g166(.a(new_n260), .b(\b[22] ), .c(new_n261), .d(\b[23] ), .out0(new_n262));
  nano22aa1d15x5               g167(.a(new_n226), .b(new_n245), .c(new_n262), .out0(new_n263));
  aoai13aa1n04x5               g168(.a(new_n262), .b(new_n247), .c(new_n232), .d(new_n245), .o1(new_n264));
  oaib12aa1n02x5               g169(.a(new_n257), .b(new_n261), .c(\b[23] ), .out0(new_n265));
  nanp02aa1n02x5               g170(.a(new_n264), .b(new_n265), .o1(new_n266));
  xnrc02aa1n12x5               g171(.a(\b[24] ), .b(\a[25] ), .out0(new_n267));
  inv000aa1d42x5               g172(.a(new_n267), .o1(new_n268));
  aoai13aa1n06x5               g173(.a(new_n268), .b(new_n266), .c(new_n194), .d(new_n263), .o1(new_n269));
  aoi112aa1n02x5               g174(.a(new_n268), .b(new_n266), .c(new_n194), .d(new_n263), .o1(new_n270));
  norb02aa1n03x4               g175(.a(new_n269), .b(new_n270), .out0(\s[25] ));
  norp02aa1n02x5               g176(.a(\b[24] ), .b(\a[25] ), .o1(new_n272));
  inv000aa1d42x5               g177(.a(new_n272), .o1(new_n273));
  xorc02aa1n02x5               g178(.a(\a[26] ), .b(\b[25] ), .out0(new_n274));
  and002aa1n02x5               g179(.a(\b[25] ), .b(\a[26] ), .o(new_n275));
  oai022aa1n02x5               g180(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n276));
  nona22aa1n02x5               g181(.a(new_n269), .b(new_n275), .c(new_n276), .out0(new_n277));
  aoai13aa1n03x5               g182(.a(new_n277), .b(new_n274), .c(new_n273), .d(new_n269), .o1(\s[26] ));
  norb02aa1n02x5               g183(.a(new_n274), .b(new_n267), .out0(new_n279));
  nand02aa1d04x5               g184(.a(new_n263), .b(new_n279), .o1(new_n280));
  aoi012aa1n12x5               g185(.a(new_n280), .b(new_n186), .c(new_n193), .o1(new_n281));
  inv000aa1n02x5               g186(.a(new_n279), .o1(new_n282));
  aob012aa1n02x5               g187(.a(new_n276), .b(\b[25] ), .c(\a[26] ), .out0(new_n283));
  aoai13aa1n12x5               g188(.a(new_n283), .b(new_n282), .c(new_n264), .d(new_n265), .o1(new_n284));
  xorc02aa1n02x5               g189(.a(\a[27] ), .b(\b[26] ), .out0(new_n285));
  nanb02aa1n02x5               g190(.a(new_n285), .b(new_n283), .out0(new_n286));
  aoi112aa1n02x5               g191(.a(new_n281), .b(new_n286), .c(new_n266), .d(new_n279), .o1(new_n287));
  oaoi13aa1n02x7               g192(.a(new_n287), .b(new_n285), .c(new_n281), .d(new_n284), .o1(\s[27] ));
  norp02aa1n02x5               g193(.a(\b[26] ), .b(\a[27] ), .o1(new_n289));
  inv000aa1d42x5               g194(.a(new_n289), .o1(new_n290));
  tech160nm_fioai012aa1n04x5   g195(.a(new_n285), .b(new_n284), .c(new_n281), .o1(new_n291));
  xorc02aa1n02x5               g196(.a(\a[28] ), .b(\b[27] ), .out0(new_n292));
  oai022aa1d24x5               g197(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n293));
  aoi012aa1n02x5               g198(.a(new_n293), .b(\a[28] ), .c(\b[27] ), .o1(new_n294));
  tech160nm_finand02aa1n03p5x5 g199(.a(new_n291), .b(new_n294), .o1(new_n295));
  aoai13aa1n03x5               g200(.a(new_n295), .b(new_n292), .c(new_n291), .d(new_n290), .o1(\s[28] ));
  xorc02aa1n02x5               g201(.a(\a[29] ), .b(\b[28] ), .out0(new_n297));
  and002aa1n02x5               g202(.a(new_n292), .b(new_n285), .o(new_n298));
  oaih12aa1n02x5               g203(.a(new_n298), .b(new_n284), .c(new_n281), .o1(new_n299));
  inv000aa1d42x5               g204(.a(\b[27] ), .o1(new_n300));
  oaib12aa1n09x5               g205(.a(new_n293), .b(new_n300), .c(\a[28] ), .out0(new_n301));
  nanp03aa1n03x5               g206(.a(new_n299), .b(new_n301), .c(new_n297), .o1(new_n302));
  inv000aa1d42x5               g207(.a(new_n301), .o1(new_n303));
  oaoi13aa1n03x5               g208(.a(new_n303), .b(new_n298), .c(new_n284), .d(new_n281), .o1(new_n304));
  oaih12aa1n02x5               g209(.a(new_n302), .b(new_n304), .c(new_n297), .o1(\s[29] ));
  xorb03aa1n02x5               g210(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g211(.a(new_n285), .b(new_n297), .c(new_n292), .o(new_n307));
  tech160nm_fioaoi03aa1n03p5x5 g212(.a(\a[29] ), .b(\b[28] ), .c(new_n301), .o1(new_n308));
  oaoi13aa1n03x5               g213(.a(new_n308), .b(new_n307), .c(new_n284), .d(new_n281), .o1(new_n309));
  xorc02aa1n02x5               g214(.a(\a[30] ), .b(\b[29] ), .out0(new_n310));
  oaih12aa1n02x5               g215(.a(new_n307), .b(new_n284), .c(new_n281), .o1(new_n311));
  norb02aa1n03x5               g216(.a(new_n310), .b(new_n308), .out0(new_n312));
  nand42aa1n03x5               g217(.a(new_n311), .b(new_n312), .o1(new_n313));
  oaih12aa1n02x5               g218(.a(new_n313), .b(new_n309), .c(new_n310), .o1(\s[30] ));
  and003aa1n02x5               g219(.a(new_n298), .b(new_n310), .c(new_n297), .o(new_n315));
  aoi012aa1n02x5               g220(.a(new_n312), .b(\a[30] ), .c(\b[29] ), .o1(new_n316));
  oaoi13aa1n03x5               g221(.a(new_n316), .b(new_n315), .c(new_n284), .d(new_n281), .o1(new_n317));
  xorc02aa1n02x5               g222(.a(\a[31] ), .b(\b[30] ), .out0(new_n318));
  oaih12aa1n02x5               g223(.a(new_n315), .b(new_n284), .c(new_n281), .o1(new_n319));
  norb02aa1n02x5               g224(.a(new_n318), .b(new_n316), .out0(new_n320));
  nand42aa1n03x5               g225(.a(new_n319), .b(new_n320), .o1(new_n321));
  oaih12aa1n02x5               g226(.a(new_n321), .b(new_n317), .c(new_n318), .o1(\s[31] ));
  xorb03aa1n02x5               g227(.a(new_n104), .b(\b[2] ), .c(new_n110), .out0(\s[3] ));
  nanb02aa1n02x5               g228(.a(new_n105), .b(new_n106), .out0(new_n324));
  oao003aa1n02x5               g229(.a(\a[3] ), .b(\b[2] ), .c(new_n104), .carry(new_n325));
  aboi22aa1n03x5               g230(.a(new_n105), .b(new_n113), .c(new_n325), .d(new_n324), .out0(\s[4] ));
  oaoi13aa1n12x5               g231(.a(new_n115), .b(new_n112), .c(new_n109), .d(new_n104), .o1(new_n327));
  oai112aa1n02x5               g232(.a(new_n112), .b(new_n115), .c(new_n109), .d(new_n104), .o1(new_n328));
  norb02aa1n02x5               g233(.a(new_n328), .b(new_n327), .out0(\s[5] ));
  inv000aa1d42x5               g234(.a(new_n114), .o1(new_n330));
  inv000aa1d42x5               g235(.a(new_n124), .o1(new_n331));
  inv000aa1d42x5               g236(.a(new_n327), .o1(new_n332));
  xnbna2aa1n03x5               g237(.a(new_n330), .b(new_n332), .c(new_n331), .out0(\s[6] ));
  norb02aa1n02x5               g238(.a(new_n119), .b(new_n118), .out0(new_n334));
  tech160nm_fioai012aa1n05x5   g239(.a(new_n330), .b(new_n327), .c(new_n124), .o1(new_n335));
  oaib12aa1n06x5               g240(.a(new_n335), .b(\b[5] ), .c(new_n122), .out0(new_n336));
  aboi22aa1n03x5               g241(.a(new_n118), .b(new_n119), .c(new_n122), .d(new_n123), .out0(new_n337));
  aoi022aa1n02x5               g242(.a(new_n336), .b(new_n334), .c(new_n335), .d(new_n337), .o1(\s[7] ));
  norb02aa1n02x5               g243(.a(new_n116), .b(new_n117), .out0(new_n339));
  aoi012aa1n02x5               g244(.a(new_n118), .b(new_n336), .c(new_n119), .o1(new_n340));
  aobi12aa1n02x5               g245(.a(new_n126), .b(new_n336), .c(new_n334), .out0(new_n341));
  oabi12aa1n02x7               g246(.a(new_n341), .b(new_n339), .c(new_n340), .out0(\s[8] ));
  xnrb03aa1n02x5               g247(.a(new_n128), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule

