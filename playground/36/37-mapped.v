// Benchmark "adder" written by ABC on Thu Jul 18 06:44:39 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n158, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n269, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n313, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n320, new_n321, new_n324,
    new_n325, new_n328, new_n329, new_n331, new_n333, new_n334, new_n335;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  orn002aa1n03x5               g002(.a(\a[2] ), .b(\b[1] ), .o(new_n98));
  nand42aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand22aa1n12x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  aob012aa1n06x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .out0(new_n101));
  nand42aa1n08x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nor002aa1n04x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  norb02aa1n06x4               g008(.a(new_n102), .b(new_n103), .out0(new_n104));
  nor042aa1n04x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand02aa1n04x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  norb02aa1n03x5               g011(.a(new_n106), .b(new_n105), .out0(new_n107));
  nand23aa1n06x5               g012(.a(new_n101), .b(new_n104), .c(new_n107), .o1(new_n108));
  tech160nm_fioai012aa1n03p5x5 g013(.a(new_n102), .b(new_n105), .c(new_n103), .o1(new_n109));
  nor042aa1n06x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nand22aa1n09x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nand22aa1n09x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nor042aa1n06x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nano23aa1n09x5               g018(.a(new_n113), .b(new_n110), .c(new_n111), .d(new_n112), .out0(new_n114));
  nand02aa1d24x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nor042aa1n03x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nor042aa1d18x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  nand02aa1n08x5               g022(.a(\b[4] ), .b(\a[5] ), .o1(new_n118));
  nano23aa1n06x5               g023(.a(new_n117), .b(new_n116), .c(new_n118), .d(new_n115), .out0(new_n119));
  nand22aa1n03x5               g024(.a(new_n119), .b(new_n114), .o1(new_n120));
  inv040aa1n04x5               g025(.a(new_n117), .o1(new_n121));
  oaoi03aa1n12x5               g026(.a(\a[6] ), .b(\b[5] ), .c(new_n121), .o1(new_n122));
  ao0012aa1n03x5               g027(.a(new_n110), .b(new_n113), .c(new_n111), .o(new_n123));
  aoi012aa1n06x5               g028(.a(new_n123), .b(new_n114), .c(new_n122), .o1(new_n124));
  aoai13aa1n12x5               g029(.a(new_n124), .b(new_n120), .c(new_n108), .d(new_n109), .o1(new_n125));
  tech160nm_fixorc02aa1n03p5x5 g030(.a(\a[9] ), .b(\b[8] ), .out0(new_n126));
  nand02aa1d06x5               g031(.a(new_n125), .b(new_n126), .o1(new_n127));
  tech160nm_fixorc02aa1n02p5x5 g032(.a(\a[10] ), .b(\b[9] ), .out0(new_n128));
  oaih22aa1n04x5               g033(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n129));
  aoi012aa1n02x5               g034(.a(new_n129), .b(\a[10] ), .c(\b[9] ), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(new_n127), .b(new_n130), .o1(new_n131));
  aoai13aa1n02x5               g036(.a(new_n131), .b(new_n128), .c(new_n97), .d(new_n127), .o1(\s[10] ));
  nanb02aa1n06x5               g037(.a(new_n129), .b(new_n127), .out0(new_n133));
  nor002aa1n04x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  aoi022aa1n06x5               g039(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  tech160nm_fixnrc02aa1n02p5x5 g041(.a(\b[10] ), .b(\a[11] ), .out0(new_n137));
  aob012aa1n02x5               g042(.a(new_n133), .b(\b[9] ), .c(\a[10] ), .out0(new_n138));
  aoi022aa1n02x5               g043(.a(new_n138), .b(new_n137), .c(new_n133), .d(new_n136), .o1(\s[11] ));
  tech160nm_fiaoi012aa1n03p5x5 g044(.a(new_n134), .b(new_n133), .c(new_n135), .o1(new_n140));
  xnrc02aa1n12x5               g045(.a(\b[11] ), .b(\a[12] ), .out0(new_n141));
  inv000aa1d42x5               g046(.a(new_n141), .o1(new_n142));
  aoi112aa1n06x5               g047(.a(new_n134), .b(new_n142), .c(new_n133), .d(new_n135), .o1(new_n143));
  oab012aa1n03x5               g048(.a(new_n143), .b(new_n140), .c(new_n141), .out0(\s[12] ));
  nano23aa1n03x7               g049(.a(new_n141), .b(new_n137), .c(new_n128), .d(new_n126), .out0(new_n145));
  oab012aa1n02x4               g050(.a(new_n134), .b(\a[12] ), .c(\b[11] ), .out0(new_n146));
  aobi12aa1n03x7               g051(.a(new_n146), .b(new_n135), .c(new_n129), .out0(new_n147));
  aoi012aa1n02x7               g052(.a(new_n147), .b(\a[12] ), .c(\b[11] ), .o1(new_n148));
  nor042aa1n04x5               g053(.a(\b[12] ), .b(\a[13] ), .o1(new_n149));
  nand02aa1d16x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  norb02aa1n02x5               g055(.a(new_n150), .b(new_n149), .out0(new_n151));
  aoai13aa1n03x5               g056(.a(new_n151), .b(new_n148), .c(new_n125), .d(new_n145), .o1(new_n152));
  aoi112aa1n02x5               g057(.a(new_n151), .b(new_n148), .c(new_n125), .d(new_n145), .o1(new_n153));
  norb02aa1n02x5               g058(.a(new_n152), .b(new_n153), .out0(\s[13] ));
  orn002aa1n02x5               g059(.a(\a[13] ), .b(\b[12] ), .o(new_n155));
  nor002aa1n04x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nanp02aa1n24x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  norb02aa1n02x5               g062(.a(new_n157), .b(new_n156), .out0(new_n158));
  xnbna2aa1n03x5               g063(.a(new_n158), .b(new_n152), .c(new_n155), .out0(\s[14] ));
  nano23aa1n06x5               g064(.a(new_n149), .b(new_n156), .c(new_n157), .d(new_n150), .out0(new_n160));
  aoai13aa1n06x5               g065(.a(new_n160), .b(new_n148), .c(new_n125), .d(new_n145), .o1(new_n161));
  aoi012aa1n06x5               g066(.a(new_n156), .b(new_n149), .c(new_n157), .o1(new_n162));
  inv040aa1d32x5               g067(.a(\a[15] ), .o1(new_n163));
  inv040aa1d30x5               g068(.a(\b[14] ), .o1(new_n164));
  nand02aa1d06x5               g069(.a(new_n164), .b(new_n163), .o1(new_n165));
  nand02aa1n04x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  nand22aa1n09x5               g071(.a(new_n165), .b(new_n166), .o1(new_n167));
  inv000aa1d42x5               g072(.a(new_n167), .o1(new_n168));
  xnbna2aa1n03x5               g073(.a(new_n168), .b(new_n161), .c(new_n162), .out0(\s[15] ));
  nanp02aa1n02x5               g074(.a(new_n161), .b(new_n162), .o1(new_n170));
  nanp02aa1n02x5               g075(.a(new_n170), .b(new_n168), .o1(new_n171));
  aoai13aa1n03x5               g076(.a(new_n165), .b(new_n167), .c(new_n161), .d(new_n162), .o1(new_n172));
  xorc02aa1n12x5               g077(.a(\a[16] ), .b(\b[15] ), .out0(new_n173));
  norb02aa1n02x5               g078(.a(new_n165), .b(new_n173), .out0(new_n174));
  aoi022aa1n03x5               g079(.a(new_n172), .b(new_n173), .c(new_n171), .d(new_n174), .o1(\s[16] ));
  nor042aa1n03x5               g080(.a(new_n141), .b(new_n137), .o1(new_n176));
  nand23aa1n02x5               g081(.a(new_n160), .b(new_n168), .c(new_n173), .o1(new_n177));
  nano32aa1d12x5               g082(.a(new_n177), .b(new_n176), .c(new_n128), .d(new_n126), .out0(new_n178));
  nanp02aa1n09x5               g083(.a(new_n125), .b(new_n178), .o1(new_n179));
  nanp02aa1n02x5               g084(.a(\b[15] ), .b(\a[16] ), .o1(new_n180));
  aob012aa1n02x5               g085(.a(new_n180), .b(\b[11] ), .c(\a[12] ), .out0(new_n181));
  nona23aa1n09x5               g086(.a(new_n160), .b(new_n173), .c(new_n181), .d(new_n167), .out0(new_n182));
  oa0022aa1n02x5               g087(.a(\a[16] ), .b(\b[15] ), .c(\a[15] ), .d(\b[14] ), .o(new_n183));
  aoai13aa1n02x5               g088(.a(new_n183), .b(new_n162), .c(\a[15] ), .d(\b[14] ), .o1(new_n184));
  nanp02aa1n02x5               g089(.a(new_n184), .b(new_n180), .o1(new_n185));
  oai012aa1n12x5               g090(.a(new_n185), .b(new_n182), .c(new_n147), .o1(new_n186));
  inv000aa1n06x5               g091(.a(new_n186), .o1(new_n187));
  xorc02aa1n12x5               g092(.a(\a[17] ), .b(\b[16] ), .out0(new_n188));
  xnbna2aa1n03x5               g093(.a(new_n188), .b(new_n179), .c(new_n187), .out0(\s[17] ));
  inv000aa1d42x5               g094(.a(\a[17] ), .o1(new_n190));
  nanb02aa1n02x5               g095(.a(\b[16] ), .b(new_n190), .out0(new_n191));
  aoai13aa1n03x5               g096(.a(new_n188), .b(new_n186), .c(new_n125), .d(new_n178), .o1(new_n192));
  nor042aa1n04x5               g097(.a(\b[17] ), .b(\a[18] ), .o1(new_n193));
  nand42aa1n20x5               g098(.a(\b[17] ), .b(\a[18] ), .o1(new_n194));
  norb02aa1n03x5               g099(.a(new_n194), .b(new_n193), .out0(new_n195));
  xnbna2aa1n03x5               g100(.a(new_n195), .b(new_n192), .c(new_n191), .out0(\s[18] ));
  and002aa1n06x5               g101(.a(new_n188), .b(new_n195), .o(new_n197));
  aoai13aa1n06x5               g102(.a(new_n197), .b(new_n186), .c(new_n125), .d(new_n178), .o1(new_n198));
  nor042aa1n04x5               g103(.a(\b[16] ), .b(\a[17] ), .o1(new_n199));
  aoi012aa1n02x5               g104(.a(new_n193), .b(new_n199), .c(new_n194), .o1(new_n200));
  inv040aa1d32x5               g105(.a(\a[19] ), .o1(new_n201));
  inv000aa1d42x5               g106(.a(\b[18] ), .o1(new_n202));
  nand22aa1n09x5               g107(.a(new_n202), .b(new_n201), .o1(new_n203));
  nand22aa1n04x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  nand02aa1n04x5               g109(.a(new_n203), .b(new_n204), .o1(new_n205));
  inv040aa1n02x5               g110(.a(new_n205), .o1(new_n206));
  xnbna2aa1n03x5               g111(.a(new_n206), .b(new_n198), .c(new_n200), .out0(\s[19] ));
  xnrc02aa1n02x5               g112(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanp02aa1n02x5               g113(.a(new_n198), .b(new_n200), .o1(new_n209));
  nanp02aa1n02x5               g114(.a(new_n209), .b(new_n206), .o1(new_n210));
  xorc02aa1n12x5               g115(.a(\a[20] ), .b(\b[19] ), .out0(new_n211));
  norb02aa1n02x5               g116(.a(new_n203), .b(new_n211), .out0(new_n212));
  aoai13aa1n03x5               g117(.a(new_n203), .b(new_n205), .c(new_n198), .d(new_n200), .o1(new_n213));
  aoi022aa1n03x5               g118(.a(new_n213), .b(new_n211), .c(new_n210), .d(new_n212), .o1(\s[20] ));
  nand23aa1d12x5               g119(.a(new_n197), .b(new_n206), .c(new_n211), .o1(new_n215));
  inv000aa1d42x5               g120(.a(new_n215), .o1(new_n216));
  aoai13aa1n06x5               g121(.a(new_n216), .b(new_n186), .c(new_n125), .d(new_n178), .o1(new_n217));
  aoai13aa1n04x5               g122(.a(new_n204), .b(new_n193), .c(new_n199), .d(new_n194), .o1(new_n218));
  oa0022aa1n02x5               g123(.a(\a[20] ), .b(\b[19] ), .c(\a[19] ), .d(\b[18] ), .o(new_n219));
  aoi022aa1n02x5               g124(.a(new_n218), .b(new_n219), .c(\b[19] ), .d(\a[20] ), .o1(new_n220));
  inv000aa1n02x5               g125(.a(new_n220), .o1(new_n221));
  nor002aa1d32x5               g126(.a(\b[20] ), .b(\a[21] ), .o1(new_n222));
  nand42aa1n03x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  norb02aa1n02x5               g128(.a(new_n223), .b(new_n222), .out0(new_n224));
  xnbna2aa1n03x5               g129(.a(new_n224), .b(new_n217), .c(new_n221), .out0(\s[21] ));
  nand02aa1d06x5               g130(.a(new_n179), .b(new_n187), .o1(new_n226));
  aoai13aa1n06x5               g131(.a(new_n224), .b(new_n220), .c(new_n226), .d(new_n216), .o1(new_n227));
  orn002aa1n24x5               g132(.a(\a[22] ), .b(\b[21] ), .o(new_n228));
  nand02aa1d28x5               g133(.a(\b[21] ), .b(\a[22] ), .o1(new_n229));
  nand02aa1d24x5               g134(.a(new_n228), .b(new_n229), .o1(new_n230));
  inv000aa1d42x5               g135(.a(new_n230), .o1(new_n231));
  aoi012aa1n02x5               g136(.a(new_n222), .b(new_n228), .c(new_n229), .o1(new_n232));
  inv000aa1d42x5               g137(.a(new_n222), .o1(new_n233));
  inv000aa1d42x5               g138(.a(new_n224), .o1(new_n234));
  aoai13aa1n02x5               g139(.a(new_n233), .b(new_n234), .c(new_n217), .d(new_n221), .o1(new_n235));
  aoi022aa1n03x5               g140(.a(new_n235), .b(new_n231), .c(new_n227), .d(new_n232), .o1(\s[22] ));
  nano22aa1n03x7               g141(.a(new_n230), .b(new_n233), .c(new_n223), .out0(new_n237));
  norb02aa1n06x5               g142(.a(new_n237), .b(new_n215), .out0(new_n238));
  aoai13aa1n06x5               g143(.a(new_n238), .b(new_n186), .c(new_n125), .d(new_n178), .o1(new_n239));
  nanp02aa1n06x5               g144(.a(new_n218), .b(new_n219), .o1(new_n240));
  nanp02aa1n02x5               g145(.a(\b[19] ), .b(\a[20] ), .o1(new_n241));
  nano32aa1n03x7               g146(.a(new_n230), .b(new_n233), .c(new_n223), .d(new_n241), .out0(new_n242));
  oaoi03aa1n02x5               g147(.a(\a[22] ), .b(\b[21] ), .c(new_n233), .o1(new_n243));
  aoi012aa1n02x5               g148(.a(new_n243), .b(new_n240), .c(new_n242), .o1(new_n244));
  nanp02aa1n03x5               g149(.a(new_n239), .b(new_n244), .o1(new_n245));
  xorc02aa1n02x5               g150(.a(\a[23] ), .b(\b[22] ), .out0(new_n246));
  aoi112aa1n02x5               g151(.a(new_n246), .b(new_n243), .c(new_n240), .d(new_n242), .o1(new_n247));
  aoi022aa1n02x5               g152(.a(new_n245), .b(new_n246), .c(new_n239), .d(new_n247), .o1(\s[23] ));
  nanp02aa1n03x5               g153(.a(new_n245), .b(new_n246), .o1(new_n249));
  xorc02aa1n02x5               g154(.a(\a[24] ), .b(\b[23] ), .out0(new_n250));
  nor042aa1n09x5               g155(.a(\b[22] ), .b(\a[23] ), .o1(new_n251));
  norp02aa1n02x5               g156(.a(new_n250), .b(new_n251), .o1(new_n252));
  inv000aa1d42x5               g157(.a(new_n251), .o1(new_n253));
  inv000aa1d42x5               g158(.a(new_n246), .o1(new_n254));
  aoai13aa1n02x5               g159(.a(new_n253), .b(new_n254), .c(new_n239), .d(new_n244), .o1(new_n255));
  aoi022aa1n03x5               g160(.a(new_n255), .b(new_n250), .c(new_n249), .d(new_n252), .o1(\s[24] ));
  nanp02aa1n02x5               g161(.a(\b[22] ), .b(\a[23] ), .o1(new_n257));
  xnrc02aa1n02x5               g162(.a(\b[23] ), .b(\a[24] ), .out0(new_n258));
  nano22aa1n06x5               g163(.a(new_n258), .b(new_n253), .c(new_n257), .out0(new_n259));
  nano22aa1n03x7               g164(.a(new_n215), .b(new_n237), .c(new_n259), .out0(new_n260));
  aoai13aa1n06x5               g165(.a(new_n260), .b(new_n186), .c(new_n125), .d(new_n178), .o1(new_n261));
  aoai13aa1n06x5               g166(.a(new_n259), .b(new_n243), .c(new_n240), .d(new_n242), .o1(new_n262));
  oao003aa1n02x5               g167(.a(\a[24] ), .b(\b[23] ), .c(new_n253), .carry(new_n263));
  nand02aa1d10x5               g168(.a(new_n262), .b(new_n263), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n264), .o1(new_n265));
  xnrc02aa1n12x5               g170(.a(\b[24] ), .b(\a[25] ), .out0(new_n266));
  inv000aa1d42x5               g171(.a(new_n266), .o1(new_n267));
  xnbna2aa1n03x5               g172(.a(new_n267), .b(new_n261), .c(new_n265), .out0(\s[25] ));
  aoai13aa1n06x5               g173(.a(new_n267), .b(new_n264), .c(new_n226), .d(new_n260), .o1(new_n269));
  tech160nm_fixorc02aa1n05x5   g174(.a(\a[26] ), .b(\b[25] ), .out0(new_n270));
  norp02aa1n02x5               g175(.a(\b[24] ), .b(\a[25] ), .o1(new_n271));
  norp02aa1n02x5               g176(.a(new_n270), .b(new_n271), .o1(new_n272));
  inv000aa1d42x5               g177(.a(new_n271), .o1(new_n273));
  aoai13aa1n02x5               g178(.a(new_n273), .b(new_n266), .c(new_n261), .d(new_n265), .o1(new_n274));
  aoi022aa1n03x5               g179(.a(new_n274), .b(new_n270), .c(new_n269), .d(new_n272), .o1(\s[26] ));
  norb02aa1d21x5               g180(.a(new_n270), .b(new_n266), .out0(new_n276));
  nano32aa1n03x7               g181(.a(new_n215), .b(new_n276), .c(new_n237), .d(new_n259), .out0(new_n277));
  aoai13aa1n09x5               g182(.a(new_n277), .b(new_n186), .c(new_n125), .d(new_n178), .o1(new_n278));
  nanp02aa1n02x5               g183(.a(\b[25] ), .b(\a[26] ), .o1(new_n279));
  oai022aa1n02x5               g184(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n280));
  aoi022aa1n12x5               g185(.a(new_n264), .b(new_n276), .c(new_n279), .d(new_n280), .o1(new_n281));
  xorc02aa1n12x5               g186(.a(\a[27] ), .b(\b[26] ), .out0(new_n282));
  xnbna2aa1n03x5               g187(.a(new_n282), .b(new_n281), .c(new_n278), .out0(\s[27] ));
  inv000aa1d42x5               g188(.a(new_n276), .o1(new_n284));
  nanp02aa1n02x5               g189(.a(new_n280), .b(new_n279), .o1(new_n285));
  aoai13aa1n12x5               g190(.a(new_n285), .b(new_n284), .c(new_n262), .d(new_n263), .o1(new_n286));
  aoai13aa1n02x7               g191(.a(new_n282), .b(new_n286), .c(new_n226), .d(new_n277), .o1(new_n287));
  xorc02aa1n02x5               g192(.a(\a[28] ), .b(\b[27] ), .out0(new_n288));
  norp02aa1n02x5               g193(.a(\b[26] ), .b(\a[27] ), .o1(new_n289));
  norp02aa1n02x5               g194(.a(new_n288), .b(new_n289), .o1(new_n290));
  inv000aa1n03x5               g195(.a(new_n289), .o1(new_n291));
  inv000aa1n02x5               g196(.a(new_n282), .o1(new_n292));
  aoai13aa1n03x5               g197(.a(new_n291), .b(new_n292), .c(new_n281), .d(new_n278), .o1(new_n293));
  aoi022aa1n03x5               g198(.a(new_n293), .b(new_n288), .c(new_n287), .d(new_n290), .o1(\s[28] ));
  and002aa1n06x5               g199(.a(new_n288), .b(new_n282), .o(new_n295));
  aoai13aa1n02x7               g200(.a(new_n295), .b(new_n286), .c(new_n226), .d(new_n277), .o1(new_n296));
  inv000aa1d42x5               g201(.a(new_n295), .o1(new_n297));
  oao003aa1n02x5               g202(.a(\a[28] ), .b(\b[27] ), .c(new_n291), .carry(new_n298));
  aoai13aa1n03x5               g203(.a(new_n298), .b(new_n297), .c(new_n281), .d(new_n278), .o1(new_n299));
  xorc02aa1n02x5               g204(.a(\a[29] ), .b(\b[28] ), .out0(new_n300));
  norb02aa1n02x5               g205(.a(new_n298), .b(new_n300), .out0(new_n301));
  aoi022aa1n03x5               g206(.a(new_n299), .b(new_n300), .c(new_n296), .d(new_n301), .o1(\s[29] ));
  xorb03aa1n02x5               g207(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n03x7               g208(.a(new_n292), .b(new_n288), .c(new_n300), .out0(new_n304));
  aoai13aa1n02x7               g209(.a(new_n304), .b(new_n286), .c(new_n226), .d(new_n277), .o1(new_n305));
  inv000aa1d42x5               g210(.a(new_n304), .o1(new_n306));
  oaoi03aa1n02x5               g211(.a(\a[29] ), .b(\b[28] ), .c(new_n298), .o1(new_n307));
  inv000aa1n03x5               g212(.a(new_n307), .o1(new_n308));
  aoai13aa1n03x5               g213(.a(new_n308), .b(new_n306), .c(new_n281), .d(new_n278), .o1(new_n309));
  xorc02aa1n02x5               g214(.a(\a[30] ), .b(\b[29] ), .out0(new_n310));
  and002aa1n02x5               g215(.a(\b[28] ), .b(\a[29] ), .o(new_n311));
  oabi12aa1n02x5               g216(.a(new_n310), .b(\a[29] ), .c(\b[28] ), .out0(new_n312));
  oab012aa1n02x4               g217(.a(new_n312), .b(new_n298), .c(new_n311), .out0(new_n313));
  aoi022aa1n03x5               g218(.a(new_n309), .b(new_n310), .c(new_n305), .d(new_n313), .o1(\s[30] ));
  nano32aa1d15x5               g219(.a(new_n292), .b(new_n310), .c(new_n288), .d(new_n300), .out0(new_n315));
  aoai13aa1n02x7               g220(.a(new_n315), .b(new_n286), .c(new_n226), .d(new_n277), .o1(new_n316));
  xorc02aa1n02x5               g221(.a(\a[31] ), .b(\b[30] ), .out0(new_n317));
  oao003aa1n02x5               g222(.a(\a[30] ), .b(\b[29] ), .c(new_n308), .carry(new_n318));
  norb02aa1n02x5               g223(.a(new_n318), .b(new_n317), .out0(new_n319));
  inv000aa1d42x5               g224(.a(new_n315), .o1(new_n320));
  aoai13aa1n03x5               g225(.a(new_n318), .b(new_n320), .c(new_n281), .d(new_n278), .o1(new_n321));
  aoi022aa1n03x5               g226(.a(new_n321), .b(new_n317), .c(new_n316), .d(new_n319), .o1(\s[31] ));
  xorb03aa1n02x5               g227(.a(new_n101), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  nanp02aa1n02x5               g228(.a(new_n108), .b(new_n109), .o1(new_n324));
  aoi112aa1n02x5               g229(.a(new_n105), .b(new_n104), .c(new_n101), .d(new_n106), .o1(new_n325));
  oaoi13aa1n02x5               g230(.a(new_n325), .b(new_n324), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  xorb03aa1n02x5               g231(.a(new_n324), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  norb02aa1n02x5               g232(.a(new_n115), .b(new_n116), .out0(new_n328));
  nanp03aa1n02x5               g233(.a(new_n324), .b(new_n121), .c(new_n118), .o1(new_n329));
  xnbna2aa1n03x5               g234(.a(new_n328), .b(new_n329), .c(new_n121), .out0(\s[6] ));
  tech160nm_fiao0012aa1n02p5x5 g235(.a(new_n122), .b(new_n324), .c(new_n119), .o(new_n331));
  xorb03aa1n02x5               g236(.a(new_n331), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  norb02aa1n02x5               g237(.a(new_n111), .b(new_n110), .out0(new_n333));
  aoai13aa1n02x5               g238(.a(new_n333), .b(new_n113), .c(new_n331), .d(new_n112), .o1(new_n334));
  aoi112aa1n02x5               g239(.a(new_n113), .b(new_n333), .c(new_n331), .d(new_n112), .o1(new_n335));
  norb02aa1n02x5               g240(.a(new_n334), .b(new_n335), .out0(\s[8] ));
  xorb03aa1n02x5               g241(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


