// Benchmark "adder" written by ABC on Thu Jul 18 07:18:30 2024

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
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n151, new_n152, new_n153, new_n154, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n163, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n251, new_n252, new_n253,
    new_n254, new_n255, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n264, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n285, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n293, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n302, new_n303,
    new_n304, new_n307, new_n308, new_n310, new_n311, new_n312, new_n313,
    new_n315, new_n317;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n04x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv020aa1n04x5               g002(.a(new_n97), .o1(new_n98));
  nand22aa1n03x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand22aa1n09x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nor042aa1n03x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  oaih12aa1n06x5               g006(.a(new_n99), .b(new_n101), .c(new_n100), .o1(new_n102));
  nanp02aa1n04x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nor022aa1n16x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nand02aa1n06x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nor002aa1n20x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nona23aa1d18x5               g011(.a(new_n105), .b(new_n103), .c(new_n106), .d(new_n104), .out0(new_n107));
  oai012aa1n02x7               g012(.a(new_n105), .b(new_n106), .c(new_n104), .o1(new_n108));
  oai012aa1n12x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  nor002aa1n03x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nand02aa1d16x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  norb02aa1n06x5               g016(.a(new_n111), .b(new_n110), .out0(new_n112));
  nor042aa1n04x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nand02aa1d24x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  norb02aa1n06x5               g019(.a(new_n114), .b(new_n113), .out0(new_n115));
  nor002aa1d32x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nand42aa1n06x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nanb02aa1d36x5               g022(.a(new_n116), .b(new_n117), .out0(new_n118));
  tech160nm_fixnrc02aa1n05x5   g023(.a(\b[4] ), .b(\a[5] ), .out0(new_n119));
  nano23aa1d15x5               g024(.a(new_n119), .b(new_n118), .c(new_n115), .d(new_n112), .out0(new_n120));
  nor042aa1n03x5               g025(.a(\b[4] ), .b(\a[5] ), .o1(new_n121));
  aoi112aa1n03x5               g026(.a(new_n116), .b(new_n113), .c(new_n121), .d(new_n114), .o1(new_n122));
  aoi022aa1n02x5               g027(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n123));
  obai22aa1n09x5               g028(.a(new_n123), .b(new_n122), .c(\a[8] ), .d(\b[7] ), .out0(new_n124));
  tech160nm_fixorc02aa1n03p5x5 g029(.a(\a[9] ), .b(\b[8] ), .out0(new_n125));
  aoai13aa1n06x5               g030(.a(new_n125), .b(new_n124), .c(new_n120), .d(new_n109), .o1(new_n126));
  xorc02aa1n06x5               g031(.a(\a[10] ), .b(\b[9] ), .out0(new_n127));
  xnbna2aa1n03x5               g032(.a(new_n127), .b(new_n126), .c(new_n98), .out0(\s[10] ));
  nanp02aa1n03x5               g033(.a(new_n126), .b(new_n98), .o1(new_n129));
  oaoi03aa1n09x5               g034(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n130));
  norp02aa1n04x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nand42aa1d28x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  aoai13aa1n06x5               g038(.a(new_n133), .b(new_n130), .c(new_n129), .d(new_n127), .o1(new_n134));
  aoi112aa1n02x5               g039(.a(new_n133), .b(new_n130), .c(new_n129), .d(new_n127), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n134), .b(new_n135), .out0(\s[11] ));
  orn002aa1n02x5               g041(.a(\a[11] ), .b(\b[10] ), .o(new_n137));
  nor042aa1n04x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nanp02aa1n24x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  aobi12aa1n06x5               g045(.a(new_n140), .b(new_n134), .c(new_n137), .out0(new_n141));
  nona22aa1n03x5               g046(.a(new_n134), .b(new_n140), .c(new_n131), .out0(new_n142));
  norb02aa1n03x4               g047(.a(new_n142), .b(new_n141), .out0(\s[12] ));
  nano23aa1n06x5               g048(.a(new_n131), .b(new_n138), .c(new_n139), .d(new_n132), .out0(new_n144));
  and003aa1n02x5               g049(.a(new_n144), .b(new_n127), .c(new_n125), .o(new_n145));
  aoai13aa1n06x5               g050(.a(new_n145), .b(new_n124), .c(new_n120), .d(new_n109), .o1(new_n146));
  aoi112aa1n02x5               g051(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n147));
  aoi112aa1n03x5               g052(.a(new_n147), .b(new_n138), .c(new_n144), .d(new_n130), .o1(new_n148));
  xorc02aa1n12x5               g053(.a(\a[13] ), .b(\b[12] ), .out0(new_n149));
  xnbna2aa1n03x5               g054(.a(new_n149), .b(new_n146), .c(new_n148), .out0(\s[13] ));
  inv040aa1d32x5               g055(.a(\a[14] ), .o1(new_n151));
  nanp02aa1n02x5               g056(.a(new_n146), .b(new_n148), .o1(new_n152));
  nor042aa1d18x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  aoi012aa1n02x5               g058(.a(new_n153), .b(new_n152), .c(new_n149), .o1(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[13] ), .c(new_n151), .out0(\s[14] ));
  xorc02aa1n02x5               g060(.a(\a[14] ), .b(\b[13] ), .out0(new_n156));
  nanp02aa1n02x5               g061(.a(new_n156), .b(new_n149), .o1(new_n157));
  inv040aa1d32x5               g062(.a(\b[13] ), .o1(new_n158));
  oao003aa1n09x5               g063(.a(new_n151), .b(new_n158), .c(new_n153), .carry(new_n159));
  inv000aa1d42x5               g064(.a(new_n159), .o1(new_n160));
  aoai13aa1n03x5               g065(.a(new_n160), .b(new_n157), .c(new_n146), .d(new_n148), .o1(new_n161));
  xorb03aa1n02x5               g066(.a(new_n161), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n06x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  nand42aa1d28x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  norp02aa1n04x5               g069(.a(\b[15] ), .b(\a[16] ), .o1(new_n165));
  nand02aa1d24x5               g070(.a(\b[15] ), .b(\a[16] ), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n166), .b(new_n165), .out0(new_n167));
  aoai13aa1n03x5               g072(.a(new_n167), .b(new_n163), .c(new_n161), .d(new_n164), .o1(new_n168));
  aoi112aa1n02x5               g073(.a(new_n163), .b(new_n167), .c(new_n161), .d(new_n164), .o1(new_n169));
  norb02aa1n03x4               g074(.a(new_n168), .b(new_n169), .out0(\s[16] ));
  nano23aa1n06x5               g075(.a(new_n163), .b(new_n165), .c(new_n166), .d(new_n164), .out0(new_n171));
  nand23aa1n03x5               g076(.a(new_n171), .b(new_n149), .c(new_n156), .o1(new_n172));
  nano32aa1n03x7               g077(.a(new_n172), .b(new_n144), .c(new_n127), .d(new_n125), .out0(new_n173));
  aoai13aa1n12x5               g078(.a(new_n173), .b(new_n124), .c(new_n109), .d(new_n120), .o1(new_n174));
  tech160nm_finand02aa1n05x5   g079(.a(new_n144), .b(new_n130), .o1(new_n175));
  nona22aa1n03x5               g080(.a(new_n175), .b(new_n147), .c(new_n138), .out0(new_n176));
  inv020aa1n02x5               g081(.a(new_n172), .o1(new_n177));
  nanp02aa1n04x5               g082(.a(new_n163), .b(new_n166), .o1(new_n178));
  inv000aa1d42x5               g083(.a(new_n178), .o1(new_n179));
  nand42aa1n06x5               g084(.a(new_n171), .b(new_n159), .o1(new_n180));
  nona22aa1n03x5               g085(.a(new_n180), .b(new_n179), .c(new_n165), .out0(new_n181));
  aoi012aa1n12x5               g086(.a(new_n181), .b(new_n176), .c(new_n177), .o1(new_n182));
  nand02aa1d08x5               g087(.a(new_n174), .b(new_n182), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g089(.a(\a[18] ), .o1(new_n185));
  inv000aa1d42x5               g090(.a(\a[17] ), .o1(new_n186));
  inv000aa1d42x5               g091(.a(\b[16] ), .o1(new_n187));
  oaoi03aa1n02x5               g092(.a(new_n186), .b(new_n187), .c(new_n183), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[17] ), .c(new_n185), .out0(\s[18] ));
  xroi22aa1d06x4               g094(.a(new_n186), .b(\b[16] ), .c(new_n185), .d(\b[17] ), .out0(new_n190));
  nanp02aa1n02x5               g095(.a(new_n187), .b(new_n186), .o1(new_n191));
  oaoi03aa1n02x5               g096(.a(\a[18] ), .b(\b[17] ), .c(new_n191), .o1(new_n192));
  nor002aa1d32x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  nand42aa1d28x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  norb02aa1n02x5               g099(.a(new_n194), .b(new_n193), .out0(new_n195));
  aoai13aa1n06x5               g100(.a(new_n195), .b(new_n192), .c(new_n183), .d(new_n190), .o1(new_n196));
  aoi112aa1n02x5               g101(.a(new_n195), .b(new_n192), .c(new_n183), .d(new_n190), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n196), .b(new_n197), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor022aa1n08x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  nand42aa1n16x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  inv000aa1d42x5               g107(.a(new_n202), .o1(new_n203));
  oaoi13aa1n06x5               g108(.a(new_n203), .b(new_n196), .c(\a[19] ), .d(\b[18] ), .o1(new_n204));
  nona22aa1n02x5               g109(.a(new_n196), .b(new_n202), .c(new_n193), .out0(new_n205));
  norb02aa1n03x4               g110(.a(new_n205), .b(new_n204), .out0(\s[20] ));
  nano23aa1n06x5               g111(.a(new_n193), .b(new_n200), .c(new_n201), .d(new_n194), .out0(new_n207));
  nanp02aa1n02x5               g112(.a(new_n190), .b(new_n207), .o1(new_n208));
  oai022aa1n04x7               g113(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n209));
  oaib12aa1n09x5               g114(.a(new_n209), .b(new_n185), .c(\b[17] ), .out0(new_n210));
  nona23aa1n12x5               g115(.a(new_n201), .b(new_n194), .c(new_n193), .d(new_n200), .out0(new_n211));
  tech160nm_fioai012aa1n04x5   g116(.a(new_n201), .b(new_n200), .c(new_n193), .o1(new_n212));
  oai012aa1n18x5               g117(.a(new_n212), .b(new_n211), .c(new_n210), .o1(new_n213));
  inv000aa1d42x5               g118(.a(new_n213), .o1(new_n214));
  aoai13aa1n04x5               g119(.a(new_n214), .b(new_n208), .c(new_n174), .d(new_n182), .o1(new_n215));
  xorb03aa1n02x5               g120(.a(new_n215), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n02x5               g121(.a(\b[20] ), .b(\a[21] ), .o1(new_n217));
  xorc02aa1n02x5               g122(.a(\a[21] ), .b(\b[20] ), .out0(new_n218));
  xorc02aa1n02x5               g123(.a(\a[22] ), .b(\b[21] ), .out0(new_n219));
  aoai13aa1n06x5               g124(.a(new_n219), .b(new_n217), .c(new_n215), .d(new_n218), .o1(new_n220));
  aoi112aa1n03x5               g125(.a(new_n217), .b(new_n219), .c(new_n215), .d(new_n218), .o1(new_n221));
  norb02aa1n02x5               g126(.a(new_n220), .b(new_n221), .out0(\s[22] ));
  inv000aa1d42x5               g127(.a(\a[21] ), .o1(new_n223));
  inv040aa1d32x5               g128(.a(\a[22] ), .o1(new_n224));
  xroi22aa1d06x4               g129(.a(new_n223), .b(\b[20] ), .c(new_n224), .d(\b[21] ), .out0(new_n225));
  nanp03aa1n02x5               g130(.a(new_n225), .b(new_n190), .c(new_n207), .o1(new_n226));
  inv000aa1d42x5               g131(.a(\b[21] ), .o1(new_n227));
  oaoi03aa1n12x5               g132(.a(new_n224), .b(new_n227), .c(new_n217), .o1(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  aoi012aa1n02x5               g134(.a(new_n229), .b(new_n213), .c(new_n225), .o1(new_n230));
  aoai13aa1n06x5               g135(.a(new_n230), .b(new_n226), .c(new_n174), .d(new_n182), .o1(new_n231));
  xorb03aa1n02x5               g136(.a(new_n231), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g137(.a(\b[22] ), .b(\a[23] ), .o1(new_n233));
  tech160nm_fixorc02aa1n02p5x5 g138(.a(\a[23] ), .b(\b[22] ), .out0(new_n234));
  xorc02aa1n02x5               g139(.a(\a[24] ), .b(\b[23] ), .out0(new_n235));
  aoai13aa1n03x5               g140(.a(new_n235), .b(new_n233), .c(new_n231), .d(new_n234), .o1(new_n236));
  aoi112aa1n02x5               g141(.a(new_n233), .b(new_n235), .c(new_n231), .d(new_n234), .o1(new_n237));
  norb02aa1n02x5               g142(.a(new_n236), .b(new_n237), .out0(\s[24] ));
  and002aa1n02x5               g143(.a(new_n235), .b(new_n234), .o(new_n239));
  inv000aa1n02x5               g144(.a(new_n239), .o1(new_n240));
  nano32aa1n02x4               g145(.a(new_n240), .b(new_n225), .c(new_n190), .d(new_n207), .out0(new_n241));
  inv000aa1n03x5               g146(.a(new_n212), .o1(new_n242));
  aoai13aa1n06x5               g147(.a(new_n225), .b(new_n242), .c(new_n207), .d(new_n192), .o1(new_n243));
  orn002aa1n02x5               g148(.a(\a[23] ), .b(\b[22] ), .o(new_n244));
  oao003aa1n02x5               g149(.a(\a[24] ), .b(\b[23] ), .c(new_n244), .carry(new_n245));
  aoai13aa1n06x5               g150(.a(new_n245), .b(new_n240), .c(new_n243), .d(new_n228), .o1(new_n246));
  xorc02aa1n12x5               g151(.a(\a[25] ), .b(\b[24] ), .out0(new_n247));
  aoai13aa1n06x5               g152(.a(new_n247), .b(new_n246), .c(new_n183), .d(new_n241), .o1(new_n248));
  aoi112aa1n02x5               g153(.a(new_n247), .b(new_n246), .c(new_n183), .d(new_n241), .o1(new_n249));
  norb02aa1n02x5               g154(.a(new_n248), .b(new_n249), .out0(\s[25] ));
  nor042aa1n03x5               g155(.a(\b[24] ), .b(\a[25] ), .o1(new_n251));
  inv000aa1d42x5               g156(.a(new_n251), .o1(new_n252));
  tech160nm_fixorc02aa1n05x5   g157(.a(\a[26] ), .b(\b[25] ), .out0(new_n253));
  aobi12aa1n06x5               g158(.a(new_n253), .b(new_n248), .c(new_n252), .out0(new_n254));
  nona22aa1n06x5               g159(.a(new_n248), .b(new_n253), .c(new_n251), .out0(new_n255));
  norb02aa1n03x4               g160(.a(new_n255), .b(new_n254), .out0(\s[26] ));
  and002aa1n06x5               g161(.a(new_n253), .b(new_n247), .o(new_n257));
  nano22aa1n03x7               g162(.a(new_n226), .b(new_n239), .c(new_n257), .out0(new_n258));
  nand02aa1d10x5               g163(.a(new_n183), .b(new_n258), .o1(new_n259));
  oao003aa1n02x5               g164(.a(\a[26] ), .b(\b[25] ), .c(new_n252), .carry(new_n260));
  aobi12aa1n12x5               g165(.a(new_n260), .b(new_n246), .c(new_n257), .out0(new_n261));
  xorc02aa1n02x5               g166(.a(\a[27] ), .b(\b[26] ), .out0(new_n262));
  xnbna2aa1n06x5               g167(.a(new_n262), .b(new_n261), .c(new_n259), .out0(\s[27] ));
  norp02aa1n02x5               g168(.a(\b[26] ), .b(\a[27] ), .o1(new_n264));
  inv040aa1n03x5               g169(.a(new_n264), .o1(new_n265));
  aobi12aa1n06x5               g170(.a(new_n258), .b(new_n174), .c(new_n182), .out0(new_n266));
  aoai13aa1n06x5               g171(.a(new_n239), .b(new_n229), .c(new_n213), .d(new_n225), .o1(new_n267));
  inv000aa1d42x5               g172(.a(new_n257), .o1(new_n268));
  aoai13aa1n04x5               g173(.a(new_n260), .b(new_n268), .c(new_n267), .d(new_n245), .o1(new_n269));
  oaih12aa1n02x5               g174(.a(new_n262), .b(new_n269), .c(new_n266), .o1(new_n270));
  xnrc02aa1n02x5               g175(.a(\b[27] ), .b(\a[28] ), .out0(new_n271));
  tech160nm_fiaoi012aa1n02p5x5 g176(.a(new_n271), .b(new_n270), .c(new_n265), .o1(new_n272));
  aobi12aa1n06x5               g177(.a(new_n262), .b(new_n261), .c(new_n259), .out0(new_n273));
  nano22aa1n03x5               g178(.a(new_n273), .b(new_n265), .c(new_n271), .out0(new_n274));
  norp02aa1n03x5               g179(.a(new_n272), .b(new_n274), .o1(\s[28] ));
  norb02aa1n02x5               g180(.a(new_n262), .b(new_n271), .out0(new_n276));
  oaih12aa1n02x5               g181(.a(new_n276), .b(new_n269), .c(new_n266), .o1(new_n277));
  oao003aa1n02x5               g182(.a(\a[28] ), .b(\b[27] ), .c(new_n265), .carry(new_n278));
  xnrc02aa1n02x5               g183(.a(\b[28] ), .b(\a[29] ), .out0(new_n279));
  aoi012aa1n03x5               g184(.a(new_n279), .b(new_n277), .c(new_n278), .o1(new_n280));
  aobi12aa1n06x5               g185(.a(new_n276), .b(new_n261), .c(new_n259), .out0(new_n281));
  nano22aa1n03x5               g186(.a(new_n281), .b(new_n278), .c(new_n279), .out0(new_n282));
  norp02aa1n03x5               g187(.a(new_n280), .b(new_n282), .o1(\s[29] ));
  xorb03aa1n02x5               g188(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g189(.a(new_n262), .b(new_n279), .c(new_n271), .out0(new_n285));
  oaih12aa1n02x5               g190(.a(new_n285), .b(new_n269), .c(new_n266), .o1(new_n286));
  oao003aa1n02x5               g191(.a(\a[29] ), .b(\b[28] ), .c(new_n278), .carry(new_n287));
  xnrc02aa1n02x5               g192(.a(\b[29] ), .b(\a[30] ), .out0(new_n288));
  tech160nm_fiaoi012aa1n02p5x5 g193(.a(new_n288), .b(new_n286), .c(new_n287), .o1(new_n289));
  aobi12aa1n06x5               g194(.a(new_n285), .b(new_n261), .c(new_n259), .out0(new_n290));
  nano22aa1n03x5               g195(.a(new_n290), .b(new_n287), .c(new_n288), .out0(new_n291));
  norp02aa1n03x5               g196(.a(new_n289), .b(new_n291), .o1(\s[30] ));
  norb02aa1n02x5               g197(.a(new_n285), .b(new_n288), .out0(new_n293));
  aobi12aa1n06x5               g198(.a(new_n293), .b(new_n261), .c(new_n259), .out0(new_n294));
  oao003aa1n02x5               g199(.a(\a[30] ), .b(\b[29] ), .c(new_n287), .carry(new_n295));
  xnrc02aa1n02x5               g200(.a(\b[30] ), .b(\a[31] ), .out0(new_n296));
  nano22aa1n03x5               g201(.a(new_n294), .b(new_n295), .c(new_n296), .out0(new_n297));
  oaih12aa1n02x5               g202(.a(new_n293), .b(new_n269), .c(new_n266), .o1(new_n298));
  aoi012aa1n03x5               g203(.a(new_n296), .b(new_n298), .c(new_n295), .o1(new_n299));
  norp02aa1n03x5               g204(.a(new_n299), .b(new_n297), .o1(\s[31] ));
  xnrb03aa1n02x5               g205(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  norb02aa1n02x5               g206(.a(new_n105), .b(new_n106), .out0(new_n302));
  oaib12aa1n02x5               g207(.a(new_n103), .b(new_n104), .c(new_n102), .out0(new_n303));
  oai022aa1n02x5               g208(.a(new_n107), .b(new_n102), .c(\b[2] ), .d(\a[3] ), .o1(new_n304));
  mtn022aa1n02x5               g209(.a(new_n304), .b(new_n303), .sa(new_n302), .o1(\s[4] ));
  xorb03aa1n02x5               g210(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n02x5               g211(.a(new_n119), .b(new_n109), .out0(new_n307));
  oai012aa1n02x5               g212(.a(new_n307), .b(\b[4] ), .c(\a[5] ), .o1(new_n308));
  xorb03aa1n02x5               g213(.a(new_n308), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  inv000aa1d42x5               g214(.a(new_n118), .o1(new_n310));
  tech160nm_fiao0012aa1n02p5x5 g215(.a(new_n113), .b(new_n121), .c(new_n114), .o(new_n311));
  aoai13aa1n02x5               g216(.a(new_n310), .b(new_n311), .c(new_n308), .d(new_n115), .o1(new_n312));
  aoi112aa1n02x5               g217(.a(new_n311), .b(new_n310), .c(new_n308), .d(new_n115), .o1(new_n313));
  norb02aa1n02x5               g218(.a(new_n312), .b(new_n313), .out0(\s[7] ));
  inv000aa1d42x5               g219(.a(new_n116), .o1(new_n315));
  xnbna2aa1n03x5               g220(.a(new_n112), .b(new_n312), .c(new_n315), .out0(\s[8] ));
  aoi112aa1n02x5               g221(.a(new_n124), .b(new_n125), .c(new_n120), .d(new_n109), .o1(new_n317));
  norb02aa1n02x5               g222(.a(new_n126), .b(new_n317), .out0(\s[9] ));
endmodule


