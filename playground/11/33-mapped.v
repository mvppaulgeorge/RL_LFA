// Benchmark "adder" written by ABC on Wed Jul 17 17:52:55 2024

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
    new_n132, new_n133, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n207, new_n208, new_n209, new_n210, new_n211, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n312, new_n313, new_n314, new_n316, new_n318, new_n319,
    new_n321, new_n322, new_n324, new_n325, new_n326, new_n327;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  oa0022aa1n06x5               g003(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n99));
  inv040aa1d32x5               g004(.a(\a[3] ), .o1(new_n100));
  inv040aa1d32x5               g005(.a(\b[2] ), .o1(new_n101));
  nand42aa1n08x5               g006(.a(new_n101), .b(new_n100), .o1(new_n102));
  nand42aa1n04x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  tech160nm_finand02aa1n05x5   g008(.a(new_n102), .b(new_n103), .o1(new_n104));
  nand42aa1n10x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  nor002aa1d32x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  nand22aa1n12x5               g011(.a(\b[0] ), .b(\a[1] ), .o1(new_n107));
  oai012aa1d24x5               g012(.a(new_n105), .b(new_n106), .c(new_n107), .o1(new_n108));
  oai012aa1n12x5               g013(.a(new_n99), .b(new_n108), .c(new_n104), .o1(new_n109));
  nand42aa1n16x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nand02aa1d16x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  nor002aa1d32x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nanb03aa1n12x5               g017(.a(new_n112), .b(new_n110), .c(new_n111), .out0(new_n113));
  oai022aa1n12x5               g018(.a(\a[6] ), .b(\b[5] ), .c(\b[6] ), .d(\a[7] ), .o1(new_n114));
  aoi022aa1d24x5               g019(.a(\b[6] ), .b(\a[7] ), .c(\a[5] ), .d(\b[4] ), .o1(new_n115));
  inv000aa1n24x5               g020(.a(\a[5] ), .o1(new_n116));
  inv040aa1n16x5               g021(.a(\b[4] ), .o1(new_n117));
  nanp02aa1n03x5               g022(.a(new_n117), .b(new_n116), .o1(new_n118));
  nand22aa1n03x5               g023(.a(\b[3] ), .b(\a[4] ), .o1(new_n119));
  nand23aa1n06x5               g024(.a(new_n115), .b(new_n118), .c(new_n119), .o1(new_n120));
  nor043aa1n09x5               g025(.a(new_n120), .b(new_n113), .c(new_n114), .o1(new_n121));
  inv000aa1d42x5               g026(.a(new_n112), .o1(new_n122));
  inv020aa1n03x5               g027(.a(\a[7] ), .o1(new_n123));
  inv030aa1d32x5               g028(.a(\b[6] ), .o1(new_n124));
  nand02aa1n03x5               g029(.a(new_n124), .b(new_n123), .o1(new_n125));
  nor002aa1d32x5               g030(.a(\b[5] ), .b(\a[6] ), .o1(new_n126));
  aoai13aa1n06x5               g031(.a(new_n111), .b(new_n126), .c(new_n117), .d(new_n116), .o1(new_n127));
  aob012aa1d18x5               g032(.a(new_n110), .b(\b[6] ), .c(\a[7] ), .out0(new_n128));
  aoai13aa1n06x5               g033(.a(new_n122), .b(new_n128), .c(new_n127), .d(new_n125), .o1(new_n129));
  aoi012aa1d18x5               g034(.a(new_n129), .b(new_n121), .c(new_n109), .o1(new_n130));
  tech160nm_fixnrc02aa1n05x5   g035(.a(\b[8] ), .b(\a[9] ), .out0(new_n131));
  orn002aa1n02x5               g036(.a(new_n130), .b(new_n131), .o(new_n132));
  xnrc02aa1n12x5               g037(.a(\b[9] ), .b(\a[10] ), .out0(new_n133));
  xobna2aa1n03x5               g038(.a(new_n133), .b(new_n132), .c(new_n98), .out0(\s[10] ));
  orn002aa1n02x5               g039(.a(new_n131), .b(new_n133), .o(new_n135));
  nor042aa1n06x5               g040(.a(\b[9] ), .b(\a[10] ), .o1(new_n136));
  nand02aa1n04x5               g041(.a(\b[9] ), .b(\a[10] ), .o1(new_n137));
  oai012aa1d24x5               g042(.a(new_n137), .b(new_n136), .c(new_n97), .o1(new_n138));
  inv000aa1d42x5               g043(.a(new_n138), .o1(new_n139));
  oab012aa1n02x4               g044(.a(new_n139), .b(new_n130), .c(new_n135), .out0(new_n140));
  xnrb03aa1n02x5               g045(.a(new_n140), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1d32x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nand02aa1n08x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nanb02aa1n02x5               g048(.a(new_n142), .b(new_n143), .out0(new_n144));
  oaoi03aa1n02x5               g049(.a(\a[11] ), .b(\b[10] ), .c(new_n140), .o1(new_n145));
  nor002aa1d24x5               g050(.a(\b[10] ), .b(\a[11] ), .o1(new_n146));
  nand02aa1n04x5               g051(.a(\b[10] ), .b(\a[11] ), .o1(new_n147));
  nanb02aa1n02x5               g052(.a(new_n146), .b(new_n147), .out0(new_n148));
  norb03aa1n02x5               g053(.a(new_n143), .b(new_n146), .c(new_n142), .out0(new_n149));
  oai012aa1n02x5               g054(.a(new_n149), .b(new_n140), .c(new_n148), .o1(new_n150));
  aob012aa1n02x5               g055(.a(new_n150), .b(new_n145), .c(new_n144), .out0(\s[12] ));
  nona23aa1d18x5               g056(.a(new_n143), .b(new_n147), .c(new_n146), .d(new_n142), .out0(new_n152));
  nor043aa1n03x5               g057(.a(new_n152), .b(new_n133), .c(new_n131), .o1(new_n153));
  aoai13aa1n06x5               g058(.a(new_n153), .b(new_n129), .c(new_n121), .d(new_n109), .o1(new_n154));
  tech160nm_fioai012aa1n03p5x5 g059(.a(new_n143), .b(new_n142), .c(new_n146), .o1(new_n155));
  oai012aa1d24x5               g060(.a(new_n155), .b(new_n152), .c(new_n138), .o1(new_n156));
  inv000aa1d42x5               g061(.a(new_n156), .o1(new_n157));
  nor002aa1d32x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nanp02aa1n04x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  norb02aa1n02x5               g064(.a(new_n159), .b(new_n158), .out0(new_n160));
  xnbna2aa1n03x5               g065(.a(new_n160), .b(new_n154), .c(new_n157), .out0(\s[13] ));
  nanp02aa1n03x5               g066(.a(new_n154), .b(new_n157), .o1(new_n162));
  nor002aa1d32x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nand22aa1n12x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nanb02aa1n02x5               g069(.a(new_n163), .b(new_n164), .out0(new_n165));
  aoai13aa1n02x5               g070(.a(new_n165), .b(new_n158), .c(new_n162), .d(new_n159), .o1(new_n166));
  nona22aa1n02x4               g071(.a(new_n164), .b(new_n163), .c(new_n158), .out0(new_n167));
  aoai13aa1n02x5               g072(.a(new_n166), .b(new_n167), .c(new_n160), .d(new_n162), .o1(\s[14] ));
  nona23aa1n09x5               g073(.a(new_n164), .b(new_n159), .c(new_n158), .d(new_n163), .out0(new_n169));
  nanb02aa1n03x5               g074(.a(new_n169), .b(new_n162), .out0(new_n170));
  aoi012aa1n02x5               g075(.a(new_n163), .b(new_n158), .c(new_n164), .o1(new_n171));
  inv040aa1d32x5               g076(.a(\a[15] ), .o1(new_n172));
  inv000aa1d48x5               g077(.a(\b[14] ), .o1(new_n173));
  nand02aa1d04x5               g078(.a(new_n173), .b(new_n172), .o1(new_n174));
  nanp02aa1n12x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  nand22aa1n12x5               g080(.a(new_n174), .b(new_n175), .o1(new_n176));
  inv000aa1d42x5               g081(.a(new_n176), .o1(new_n177));
  xnbna2aa1n03x5               g082(.a(new_n177), .b(new_n170), .c(new_n171), .out0(\s[15] ));
  xnrc02aa1n12x5               g083(.a(\b[15] ), .b(\a[16] ), .out0(new_n179));
  aoai13aa1n02x5               g084(.a(new_n171), .b(new_n169), .c(new_n154), .d(new_n157), .o1(new_n180));
  oaoi03aa1n02x5               g085(.a(new_n172), .b(new_n173), .c(new_n180), .o1(new_n181));
  nanb02aa1n02x5               g086(.a(new_n179), .b(new_n174), .out0(new_n182));
  tech160nm_fiao0012aa1n02p5x5 g087(.a(new_n182), .b(new_n180), .c(new_n177), .o(new_n183));
  oaib12aa1n02x5               g088(.a(new_n183), .b(new_n181), .c(new_n179), .out0(\s[16] ));
  nor043aa1d12x5               g089(.a(new_n169), .b(new_n176), .c(new_n179), .o1(new_n185));
  nand02aa1d04x5               g090(.a(new_n185), .b(new_n153), .o1(new_n186));
  aob012aa1n02x5               g091(.a(new_n175), .b(\b[15] ), .c(\a[16] ), .out0(new_n187));
  aoi122aa1n06x5               g092(.a(new_n163), .b(new_n172), .c(new_n173), .d(new_n158), .e(new_n164), .o1(new_n188));
  oai022aa1n06x5               g093(.a(new_n188), .b(new_n187), .c(\b[15] ), .d(\a[16] ), .o1(new_n189));
  aoi012aa1d18x5               g094(.a(new_n189), .b(new_n156), .c(new_n185), .o1(new_n190));
  oai012aa1d24x5               g095(.a(new_n190), .b(new_n130), .c(new_n186), .o1(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g097(.a(\a[17] ), .o1(new_n193));
  inv040aa1n18x5               g098(.a(\b[16] ), .o1(new_n194));
  nand22aa1n09x5               g099(.a(new_n194), .b(new_n193), .o1(new_n195));
  oaib12aa1n06x5               g100(.a(new_n191), .b(new_n194), .c(\a[17] ), .out0(new_n196));
  xorc02aa1n02x5               g101(.a(\a[18] ), .b(\b[17] ), .out0(new_n197));
  xnbna2aa1n03x5               g102(.a(new_n197), .b(new_n196), .c(new_n195), .out0(\s[18] ));
  inv040aa1d32x5               g103(.a(\a[18] ), .o1(new_n199));
  xroi22aa1d06x4               g104(.a(new_n193), .b(\b[16] ), .c(new_n199), .d(\b[17] ), .out0(new_n200));
  oaoi03aa1n12x5               g105(.a(\a[18] ), .b(\b[17] ), .c(new_n195), .o1(new_n201));
  xorc02aa1n12x5               g106(.a(\a[19] ), .b(\b[18] ), .out0(new_n202));
  aoai13aa1n06x5               g107(.a(new_n202), .b(new_n201), .c(new_n191), .d(new_n200), .o1(new_n203));
  aoi112aa1n02x5               g108(.a(new_n202), .b(new_n201), .c(new_n191), .d(new_n200), .o1(new_n204));
  norb02aa1n02x5               g109(.a(new_n203), .b(new_n204), .out0(\s[19] ));
  xnrc02aa1n02x5               g110(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  orn002aa1n02x5               g111(.a(\a[19] ), .b(\b[18] ), .o(new_n207));
  xorc02aa1n12x5               g112(.a(\a[20] ), .b(\b[19] ), .out0(new_n208));
  nanp02aa1n02x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  oai022aa1n02x5               g114(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n210));
  nanb03aa1n03x5               g115(.a(new_n210), .b(new_n203), .c(new_n209), .out0(new_n211));
  aoai13aa1n03x5               g116(.a(new_n211), .b(new_n208), .c(new_n203), .d(new_n207), .o1(\s[20] ));
  nanp03aa1n02x5               g117(.a(new_n200), .b(new_n202), .c(new_n208), .o1(new_n213));
  nanb02aa1n06x5               g118(.a(new_n213), .b(new_n191), .out0(new_n214));
  nanp03aa1n06x5               g119(.a(new_n201), .b(new_n202), .c(new_n208), .o1(new_n215));
  nand02aa1n03x5               g120(.a(new_n210), .b(new_n209), .o1(new_n216));
  nanp02aa1n02x5               g121(.a(new_n215), .b(new_n216), .o1(new_n217));
  inv000aa1n02x5               g122(.a(new_n217), .o1(new_n218));
  xorc02aa1n12x5               g123(.a(\a[21] ), .b(\b[20] ), .out0(new_n219));
  xnbna2aa1n03x5               g124(.a(new_n219), .b(new_n214), .c(new_n218), .out0(\s[21] ));
  aobi12aa1n02x7               g125(.a(new_n219), .b(new_n214), .c(new_n218), .out0(new_n221));
  nand42aa1n02x5               g126(.a(new_n214), .b(new_n218), .o1(new_n222));
  norp02aa1n02x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  xnrc02aa1n12x5               g128(.a(\b[21] ), .b(\a[22] ), .out0(new_n224));
  aoai13aa1n03x5               g129(.a(new_n224), .b(new_n223), .c(new_n222), .d(new_n219), .o1(new_n225));
  norp02aa1n02x5               g130(.a(new_n224), .b(new_n223), .o1(new_n226));
  oaib12aa1n03x5               g131(.a(new_n225), .b(new_n221), .c(new_n226), .out0(\s[22] ));
  nanb02aa1n06x5               g132(.a(new_n224), .b(new_n219), .out0(new_n228));
  nano32aa1n02x4               g133(.a(new_n228), .b(new_n200), .c(new_n202), .d(new_n208), .out0(new_n229));
  orn002aa1n02x5               g134(.a(\a[21] ), .b(\b[20] ), .o(new_n230));
  oao003aa1n06x5               g135(.a(\a[22] ), .b(\b[21] ), .c(new_n230), .carry(new_n231));
  aoai13aa1n12x5               g136(.a(new_n231), .b(new_n228), .c(new_n215), .d(new_n216), .o1(new_n232));
  xorc02aa1n02x5               g137(.a(\a[23] ), .b(\b[22] ), .out0(new_n233));
  aoai13aa1n06x5               g138(.a(new_n233), .b(new_n232), .c(new_n191), .d(new_n229), .o1(new_n234));
  aoi112aa1n02x5               g139(.a(new_n233), .b(new_n232), .c(new_n191), .d(new_n229), .o1(new_n235));
  norb02aa1n03x4               g140(.a(new_n234), .b(new_n235), .out0(\s[23] ));
  inv000aa1d42x5               g141(.a(\a[23] ), .o1(new_n237));
  inv000aa1d42x5               g142(.a(\b[22] ), .o1(new_n238));
  nanp02aa1n02x5               g143(.a(new_n238), .b(new_n237), .o1(new_n239));
  xorc02aa1n02x5               g144(.a(\a[24] ), .b(\b[23] ), .out0(new_n240));
  nanp02aa1n02x5               g145(.a(\b[23] ), .b(\a[24] ), .o1(new_n241));
  inv000aa1n04x5               g146(.a(\a[24] ), .o1(new_n242));
  aboi22aa1n03x5               g147(.a(\b[23] ), .b(new_n242), .c(new_n237), .d(new_n238), .out0(new_n243));
  nanp03aa1n03x5               g148(.a(new_n234), .b(new_n241), .c(new_n243), .o1(new_n244));
  aoai13aa1n03x5               g149(.a(new_n244), .b(new_n240), .c(new_n239), .d(new_n234), .o1(\s[24] ));
  norb02aa1n02x5               g150(.a(new_n219), .b(new_n224), .out0(new_n246));
  xroi22aa1d06x4               g151(.a(new_n237), .b(\b[22] ), .c(new_n242), .d(\b[23] ), .out0(new_n247));
  nano22aa1n02x5               g152(.a(new_n213), .b(new_n246), .c(new_n247), .out0(new_n248));
  nand02aa1d04x5               g153(.a(new_n191), .b(new_n248), .o1(new_n249));
  oaoi03aa1n02x5               g154(.a(\a[24] ), .b(\b[23] ), .c(new_n239), .o1(new_n250));
  aoi012aa1n02x5               g155(.a(new_n250), .b(new_n232), .c(new_n247), .o1(new_n251));
  xorc02aa1n12x5               g156(.a(\a[25] ), .b(\b[24] ), .out0(new_n252));
  xnbna2aa1n03x5               g157(.a(new_n252), .b(new_n249), .c(new_n251), .out0(\s[25] ));
  aobi12aa1n02x5               g158(.a(new_n252), .b(new_n249), .c(new_n251), .out0(new_n254));
  xorc02aa1n02x5               g159(.a(\a[26] ), .b(\b[25] ), .out0(new_n255));
  nanp02aa1n03x5               g160(.a(new_n249), .b(new_n251), .o1(new_n256));
  norp02aa1n02x5               g161(.a(\b[24] ), .b(\a[25] ), .o1(new_n257));
  tech160nm_fiaoi012aa1n05x5   g162(.a(new_n257), .b(new_n256), .c(new_n252), .o1(new_n258));
  nanp02aa1n02x5               g163(.a(\b[25] ), .b(\a[26] ), .o1(new_n259));
  oai022aa1n02x5               g164(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n260));
  nanb02aa1n02x5               g165(.a(new_n260), .b(new_n259), .out0(new_n261));
  oai022aa1n03x5               g166(.a(new_n258), .b(new_n255), .c(new_n261), .d(new_n254), .o1(\s[26] ));
  and002aa1n02x5               g167(.a(new_n255), .b(new_n252), .o(new_n263));
  aoai13aa1n06x5               g168(.a(new_n263), .b(new_n250), .c(new_n232), .d(new_n247), .o1(new_n264));
  nor042aa1n06x5               g169(.a(\b[26] ), .b(\a[27] ), .o1(new_n265));
  and002aa1n02x5               g170(.a(\b[26] ), .b(\a[27] ), .o(new_n266));
  norp02aa1n02x5               g171(.a(new_n266), .b(new_n265), .o1(new_n267));
  nano32aa1n03x7               g172(.a(new_n213), .b(new_n263), .c(new_n246), .d(new_n247), .out0(new_n268));
  aoi122aa1n02x7               g173(.a(new_n267), .b(new_n259), .c(new_n260), .d(new_n191), .e(new_n268), .o1(new_n269));
  nand02aa1d06x5               g174(.a(new_n191), .b(new_n268), .o1(new_n270));
  nanp02aa1n02x5               g175(.a(new_n260), .b(new_n259), .o1(new_n271));
  nand23aa1n04x5               g176(.a(new_n264), .b(new_n270), .c(new_n271), .o1(new_n272));
  aoi022aa1n03x5               g177(.a(new_n264), .b(new_n269), .c(new_n272), .d(new_n267), .o1(\s[27] ));
  inv000aa1d42x5               g178(.a(\b[26] ), .o1(new_n274));
  oaib12aa1n03x5               g179(.a(new_n272), .b(new_n274), .c(\a[27] ), .out0(new_n275));
  inv000aa1d42x5               g180(.a(new_n265), .o1(new_n276));
  aoi022aa1n12x5               g181(.a(new_n191), .b(new_n268), .c(new_n259), .d(new_n260), .o1(new_n277));
  aoai13aa1n02x7               g182(.a(new_n276), .b(new_n266), .c(new_n277), .d(new_n264), .o1(new_n278));
  xorc02aa1n02x5               g183(.a(\a[28] ), .b(\b[27] ), .out0(new_n279));
  norp02aa1n02x5               g184(.a(new_n279), .b(new_n265), .o1(new_n280));
  aoi022aa1n03x5               g185(.a(new_n278), .b(new_n279), .c(new_n275), .d(new_n280), .o1(\s[28] ));
  inv000aa1d42x5               g186(.a(\a[28] ), .o1(new_n282));
  xroi22aa1d04x5               g187(.a(\a[27] ), .b(new_n274), .c(new_n282), .d(\b[27] ), .out0(new_n283));
  nanp02aa1n02x5               g188(.a(new_n272), .b(new_n283), .o1(new_n284));
  inv000aa1n06x5               g189(.a(new_n283), .o1(new_n285));
  oao003aa1n02x5               g190(.a(\a[28] ), .b(\b[27] ), .c(new_n276), .carry(new_n286));
  aoai13aa1n03x5               g191(.a(new_n286), .b(new_n285), .c(new_n277), .d(new_n264), .o1(new_n287));
  xorc02aa1n02x5               g192(.a(\a[29] ), .b(\b[28] ), .out0(new_n288));
  norb02aa1n02x5               g193(.a(new_n286), .b(new_n288), .out0(new_n289));
  aoi022aa1n03x5               g194(.a(new_n287), .b(new_n288), .c(new_n284), .d(new_n289), .o1(\s[29] ));
  xorb03aa1n02x5               g195(.a(new_n107), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g196(.a(new_n279), .b(new_n288), .c(new_n267), .o(new_n292));
  nanp02aa1n02x5               g197(.a(new_n272), .b(new_n292), .o1(new_n293));
  inv000aa1n02x5               g198(.a(new_n292), .o1(new_n294));
  oaoi03aa1n02x5               g199(.a(\a[29] ), .b(\b[28] ), .c(new_n286), .o1(new_n295));
  inv000aa1n03x5               g200(.a(new_n295), .o1(new_n296));
  aoai13aa1n03x5               g201(.a(new_n296), .b(new_n294), .c(new_n277), .d(new_n264), .o1(new_n297));
  xorc02aa1n02x5               g202(.a(\a[30] ), .b(\b[29] ), .out0(new_n298));
  and002aa1n02x5               g203(.a(\b[28] ), .b(\a[29] ), .o(new_n299));
  oabi12aa1n02x5               g204(.a(new_n298), .b(\a[29] ), .c(\b[28] ), .out0(new_n300));
  oab012aa1n02x4               g205(.a(new_n300), .b(new_n286), .c(new_n299), .out0(new_n301));
  aoi022aa1n03x5               g206(.a(new_n297), .b(new_n298), .c(new_n293), .d(new_n301), .o1(\s[30] ));
  nano22aa1n02x4               g207(.a(new_n285), .b(new_n288), .c(new_n298), .out0(new_n303));
  nanp02aa1n02x5               g208(.a(new_n272), .b(new_n303), .o1(new_n304));
  xorc02aa1n02x5               g209(.a(\a[31] ), .b(\b[30] ), .out0(new_n305));
  oao003aa1n02x5               g210(.a(\a[30] ), .b(\b[29] ), .c(new_n296), .carry(new_n306));
  norb02aa1n02x5               g211(.a(new_n306), .b(new_n305), .out0(new_n307));
  inv000aa1n02x5               g212(.a(new_n303), .o1(new_n308));
  aoai13aa1n03x5               g213(.a(new_n306), .b(new_n308), .c(new_n277), .d(new_n264), .o1(new_n309));
  aoi022aa1n03x5               g214(.a(new_n309), .b(new_n305), .c(new_n304), .d(new_n307), .o1(\s[31] ));
  xnbna2aa1n03x5               g215(.a(new_n108), .b(new_n102), .c(new_n103), .out0(\s[3] ));
  orn002aa1n02x5               g216(.a(new_n108), .b(new_n104), .o(new_n312));
  xorc02aa1n02x5               g217(.a(\a[4] ), .b(\b[3] ), .out0(new_n313));
  norb02aa1n02x5               g218(.a(new_n102), .b(new_n313), .out0(new_n314));
  aoi022aa1n02x5               g219(.a(new_n312), .b(new_n314), .c(new_n109), .d(new_n313), .o1(\s[4] ));
  and002aa1n06x5               g220(.a(new_n109), .b(new_n119), .o(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  inv000aa1d42x5               g222(.a(new_n126), .o1(new_n318));
  oaoi03aa1n09x5               g223(.a(new_n116), .b(new_n117), .c(new_n316), .o1(new_n319));
  xnbna2aa1n03x5               g224(.a(new_n319), .b(new_n111), .c(new_n318), .out0(\s[6] ));
  nanb03aa1n03x5               g225(.a(new_n319), .b(new_n111), .c(new_n318), .out0(new_n321));
  xorc02aa1n02x5               g226(.a(\a[7] ), .b(\b[6] ), .out0(new_n322));
  xnbna2aa1n03x5               g227(.a(new_n322), .b(new_n321), .c(new_n318), .out0(\s[7] ));
  oaoi03aa1n02x5               g228(.a(\a[6] ), .b(\b[5] ), .c(new_n319), .o1(new_n324));
  oaoi03aa1n02x5               g229(.a(new_n123), .b(new_n124), .c(new_n324), .o1(new_n325));
  nano22aa1n02x4               g230(.a(new_n112), .b(new_n125), .c(new_n110), .out0(new_n326));
  aob012aa1n02x5               g231(.a(new_n326), .b(new_n324), .c(new_n322), .out0(new_n327));
  aoai13aa1n02x5               g232(.a(new_n327), .b(new_n325), .c(new_n110), .d(new_n122), .o1(\s[8] ));
  xnrb03aa1n02x5               g233(.a(new_n130), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


