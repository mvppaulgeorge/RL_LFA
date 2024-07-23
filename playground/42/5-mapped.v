// Benchmark "adder" written by ABC on Thu Jul 18 09:29:32 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n131, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n155, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n183, new_n184, new_n185, new_n186, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n309, new_n312,
    new_n313, new_n314, new_n316, new_n317, new_n318, new_n319, new_n320,
    new_n322, new_n323;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[10] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  norp02aa1n02x5               g003(.a(\b[8] ), .b(\a[9] ), .o1(new_n99));
  nand02aa1d28x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nor042aa1n04x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  nor042aa1d18x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nand02aa1d12x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nano23aa1n09x5               g008(.a(new_n101), .b(new_n102), .c(new_n103), .d(new_n100), .out0(new_n104));
  inv040aa1d32x5               g009(.a(\a[5] ), .o1(new_n105));
  inv040aa1d32x5               g010(.a(\b[4] ), .o1(new_n106));
  nand42aa1n04x5               g011(.a(new_n106), .b(new_n105), .o1(new_n107));
  oaoi03aa1n12x5               g012(.a(\a[6] ), .b(\b[5] ), .c(new_n107), .o1(new_n108));
  oai022aa1n04x5               g013(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n109));
  aoi022aa1n09x5               g014(.a(new_n104), .b(new_n108), .c(new_n100), .d(new_n109), .o1(new_n110));
  nor042aa1n04x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  nand42aa1n08x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  norb02aa1n06x5               g017(.a(new_n112), .b(new_n111), .out0(new_n113));
  nor042aa1n03x5               g018(.a(\b[2] ), .b(\a[3] ), .o1(new_n114));
  nanp02aa1n06x5               g019(.a(\b[2] ), .b(\a[3] ), .o1(new_n115));
  norb02aa1n03x5               g020(.a(new_n115), .b(new_n114), .out0(new_n116));
  and002aa1n12x5               g021(.a(\b[1] ), .b(\a[2] ), .o(new_n117));
  nand22aa1n12x5               g022(.a(\b[0] ), .b(\a[1] ), .o1(new_n118));
  nor042aa1n06x5               g023(.a(\b[1] ), .b(\a[2] ), .o1(new_n119));
  oab012aa1n09x5               g024(.a(new_n117), .b(new_n119), .c(new_n118), .out0(new_n120));
  nanp03aa1d12x5               g025(.a(new_n120), .b(new_n113), .c(new_n116), .o1(new_n121));
  tech160nm_fioai012aa1n03p5x5 g026(.a(new_n112), .b(new_n114), .c(new_n111), .o1(new_n122));
  tech160nm_fixorc02aa1n04x5   g027(.a(\a[6] ), .b(\b[5] ), .out0(new_n123));
  xorc02aa1n12x5               g028(.a(\a[5] ), .b(\b[4] ), .out0(new_n124));
  nand23aa1n03x5               g029(.a(new_n104), .b(new_n123), .c(new_n124), .o1(new_n125));
  aoai13aa1n12x5               g030(.a(new_n110), .b(new_n125), .c(new_n121), .d(new_n122), .o1(new_n126));
  tech160nm_fioai012aa1n05x5   g031(.a(new_n98), .b(new_n126), .c(new_n99), .o1(new_n127));
  xorb03aa1n02x5               g032(.a(new_n127), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  oaoi03aa1n02x5               g033(.a(\a[10] ), .b(\b[9] ), .c(new_n127), .o1(new_n129));
  xorb03aa1n02x5               g034(.a(new_n129), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1n06x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nand22aa1n09x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nor042aa1n04x5               g037(.a(\b[11] ), .b(\a[12] ), .o1(new_n133));
  nand22aa1n12x5               g038(.a(\b[11] ), .b(\a[12] ), .o1(new_n134));
  nanb02aa1n02x5               g039(.a(new_n133), .b(new_n134), .out0(new_n135));
  aoai13aa1n02x5               g040(.a(new_n135), .b(new_n131), .c(new_n129), .d(new_n132), .o1(new_n136));
  nanp02aa1n02x5               g041(.a(\b[9] ), .b(\a[10] ), .o1(new_n137));
  xorc02aa1n02x5               g042(.a(\a[10] ), .b(\b[9] ), .out0(new_n138));
  nanp02aa1n02x5               g043(.a(new_n127), .b(new_n138), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n132), .b(new_n131), .out0(new_n140));
  nanp03aa1n02x5               g045(.a(new_n139), .b(new_n137), .c(new_n140), .o1(new_n141));
  nona22aa1n02x4               g046(.a(new_n141), .b(new_n135), .c(new_n131), .out0(new_n142));
  nanp02aa1n02x5               g047(.a(new_n136), .b(new_n142), .o1(\s[12] ));
  nano23aa1n09x5               g048(.a(new_n131), .b(new_n133), .c(new_n134), .d(new_n132), .out0(new_n144));
  orn002aa1n24x5               g049(.a(\a[9] ), .b(\b[8] ), .o(new_n145));
  oaoi03aa1n09x5               g050(.a(\a[10] ), .b(\b[9] ), .c(new_n145), .o1(new_n146));
  oai022aa1n02x5               g051(.a(\a[11] ), .b(\b[10] ), .c(\b[11] ), .d(\a[12] ), .o1(new_n147));
  aoi022aa1n09x5               g052(.a(new_n144), .b(new_n146), .c(new_n134), .d(new_n147), .o1(new_n148));
  norb02aa1n02x5               g053(.a(new_n98), .b(new_n99), .out0(new_n149));
  nano32aa1n02x4               g054(.a(new_n135), .b(new_n138), .c(new_n149), .d(new_n140), .out0(new_n150));
  nand22aa1n03x5               g055(.a(new_n126), .b(new_n150), .o1(new_n151));
  xorc02aa1n12x5               g056(.a(\a[13] ), .b(\b[12] ), .out0(new_n152));
  xnbna2aa1n03x5               g057(.a(new_n152), .b(new_n151), .c(new_n148), .out0(\s[13] ));
  aobi12aa1n02x5               g058(.a(new_n148), .b(new_n126), .c(new_n150), .out0(new_n154));
  oaoi03aa1n02x5               g059(.a(\a[13] ), .b(\b[12] ), .c(new_n154), .o1(new_n155));
  xorb03aa1n02x5               g060(.a(new_n155), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  xnrc02aa1n12x5               g061(.a(\b[13] ), .b(\a[14] ), .out0(new_n157));
  nanb02aa1n02x5               g062(.a(new_n157), .b(new_n152), .out0(new_n158));
  nanp02aa1n02x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  oai022aa1d18x5               g064(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n160));
  nand42aa1n03x5               g065(.a(new_n160), .b(new_n159), .o1(new_n161));
  aoai13aa1n06x5               g066(.a(new_n161), .b(new_n158), .c(new_n151), .d(new_n148), .o1(new_n162));
  xorb03aa1n02x5               g067(.a(new_n162), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1d32x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  nand42aa1n02x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  norb02aa1n06x5               g070(.a(new_n165), .b(new_n164), .out0(new_n166));
  nor002aa1d32x5               g071(.a(\b[15] ), .b(\a[16] ), .o1(new_n167));
  nand42aa1n06x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  nanb02aa1n12x5               g073(.a(new_n167), .b(new_n168), .out0(new_n169));
  aoai13aa1n03x5               g074(.a(new_n169), .b(new_n164), .c(new_n162), .d(new_n166), .o1(new_n170));
  nand42aa1n02x5               g075(.a(new_n162), .b(new_n166), .o1(new_n171));
  nona22aa1n02x4               g076(.a(new_n171), .b(new_n169), .c(new_n164), .out0(new_n172));
  nanp02aa1n03x5               g077(.a(new_n172), .b(new_n170), .o1(\s[16] ));
  nona23aa1n02x4               g078(.a(new_n168), .b(new_n165), .c(new_n164), .d(new_n167), .out0(new_n174));
  tech160nm_fioai012aa1n03p5x5 g079(.a(new_n168), .b(new_n167), .c(new_n164), .o1(new_n175));
  tech160nm_fioai012aa1n04x5   g080(.a(new_n175), .b(new_n174), .c(new_n161), .o1(new_n176));
  nona23aa1d18x5               g081(.a(new_n152), .b(new_n166), .c(new_n157), .d(new_n169), .out0(new_n177));
  oab012aa1n09x5               g082(.a(new_n176), .b(new_n177), .c(new_n148), .out0(new_n178));
  nano32aa1n03x7               g083(.a(new_n177), .b(new_n149), .c(new_n144), .d(new_n138), .out0(new_n179));
  nand02aa1d08x5               g084(.a(new_n126), .b(new_n179), .o1(new_n180));
  xorc02aa1n02x5               g085(.a(\a[17] ), .b(\b[16] ), .out0(new_n181));
  xnbna2aa1n03x5               g086(.a(new_n181), .b(new_n180), .c(new_n178), .out0(\s[17] ));
  inv040aa1d32x5               g087(.a(\a[18] ), .o1(new_n183));
  nanp02aa1n06x5               g088(.a(new_n180), .b(new_n178), .o1(new_n184));
  nor042aa1n04x5               g089(.a(\b[16] ), .b(\a[17] ), .o1(new_n185));
  tech160nm_fiaoi012aa1n05x5   g090(.a(new_n185), .b(new_n184), .c(new_n181), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[17] ), .c(new_n183), .out0(\s[18] ));
  oabi12aa1n06x5               g092(.a(new_n176), .b(new_n177), .c(new_n148), .out0(new_n188));
  inv000aa1d42x5               g093(.a(\a[17] ), .o1(new_n189));
  xroi22aa1d06x4               g094(.a(new_n189), .b(\b[16] ), .c(new_n183), .d(\b[17] ), .out0(new_n190));
  aoai13aa1n06x5               g095(.a(new_n190), .b(new_n188), .c(new_n126), .d(new_n179), .o1(new_n191));
  inv000aa1d42x5               g096(.a(\b[17] ), .o1(new_n192));
  oaoi03aa1n12x5               g097(.a(new_n183), .b(new_n192), .c(new_n185), .o1(new_n193));
  nor002aa1n20x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  nand22aa1n09x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  nanb02aa1n02x5               g100(.a(new_n194), .b(new_n195), .out0(new_n196));
  inv000aa1d42x5               g101(.a(new_n196), .o1(new_n197));
  xnbna2aa1n03x5               g102(.a(new_n197), .b(new_n191), .c(new_n193), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nand22aa1n03x5               g104(.a(new_n191), .b(new_n193), .o1(new_n200));
  nor042aa1n06x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  nand22aa1n09x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  nanb02aa1n02x5               g107(.a(new_n201), .b(new_n202), .out0(new_n203));
  aoai13aa1n03x5               g108(.a(new_n203), .b(new_n194), .c(new_n200), .d(new_n197), .o1(new_n204));
  nanp02aa1n02x5               g109(.a(new_n200), .b(new_n197), .o1(new_n205));
  nona22aa1n03x5               g110(.a(new_n205), .b(new_n203), .c(new_n194), .out0(new_n206));
  nanp02aa1n02x5               g111(.a(new_n206), .b(new_n204), .o1(\s[20] ));
  nona23aa1d18x5               g112(.a(new_n202), .b(new_n195), .c(new_n194), .d(new_n201), .out0(new_n208));
  oa0012aa1n12x5               g113(.a(new_n202), .b(new_n201), .c(new_n194), .o(new_n209));
  inv040aa1n08x5               g114(.a(new_n209), .o1(new_n210));
  oai012aa1d24x5               g115(.a(new_n210), .b(new_n208), .c(new_n193), .o1(new_n211));
  inv000aa1d42x5               g116(.a(new_n211), .o1(new_n212));
  nano23aa1n09x5               g117(.a(new_n194), .b(new_n201), .c(new_n202), .d(new_n195), .out0(new_n213));
  nanp02aa1n02x5               g118(.a(new_n190), .b(new_n213), .o1(new_n214));
  aoai13aa1n06x5               g119(.a(new_n212), .b(new_n214), .c(new_n180), .d(new_n178), .o1(new_n215));
  xorb03aa1n02x5               g120(.a(new_n215), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1n02x5               g121(.a(\b[20] ), .b(\a[21] ), .o1(new_n217));
  xorc02aa1n02x5               g122(.a(\a[21] ), .b(\b[20] ), .out0(new_n218));
  xorc02aa1n02x5               g123(.a(\a[22] ), .b(\b[21] ), .out0(new_n219));
  inv000aa1d42x5               g124(.a(new_n219), .o1(new_n220));
  aoai13aa1n02x7               g125(.a(new_n220), .b(new_n217), .c(new_n215), .d(new_n218), .o1(new_n221));
  nand42aa1n02x5               g126(.a(new_n215), .b(new_n218), .o1(new_n222));
  nona22aa1n03x5               g127(.a(new_n222), .b(new_n220), .c(new_n217), .out0(new_n223));
  nanp02aa1n03x5               g128(.a(new_n223), .b(new_n221), .o1(\s[22] ));
  inv000aa1d42x5               g129(.a(\a[21] ), .o1(new_n225));
  inv040aa1d32x5               g130(.a(\a[22] ), .o1(new_n226));
  xroi22aa1d06x4               g131(.a(new_n225), .b(\b[20] ), .c(new_n226), .d(\b[21] ), .out0(new_n227));
  inv000aa1d42x5               g132(.a(\b[21] ), .o1(new_n228));
  oao003aa1n02x5               g133(.a(new_n226), .b(new_n228), .c(new_n217), .carry(new_n229));
  aoi012aa1n02x5               g134(.a(new_n229), .b(new_n211), .c(new_n227), .o1(new_n230));
  nand23aa1n03x5               g135(.a(new_n227), .b(new_n190), .c(new_n213), .o1(new_n231));
  aoai13aa1n06x5               g136(.a(new_n230), .b(new_n231), .c(new_n180), .d(new_n178), .o1(new_n232));
  xorb03aa1n02x5               g137(.a(new_n232), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g138(.a(\b[22] ), .b(\a[23] ), .o1(new_n234));
  xorc02aa1n12x5               g139(.a(\a[23] ), .b(\b[22] ), .out0(new_n235));
  tech160nm_fixnrc02aa1n05x5   g140(.a(\b[23] ), .b(\a[24] ), .out0(new_n236));
  aoai13aa1n02x7               g141(.a(new_n236), .b(new_n234), .c(new_n232), .d(new_n235), .o1(new_n237));
  nand42aa1n02x5               g142(.a(new_n232), .b(new_n235), .o1(new_n238));
  nona22aa1n03x5               g143(.a(new_n238), .b(new_n236), .c(new_n234), .out0(new_n239));
  nanp02aa1n03x5               g144(.a(new_n239), .b(new_n237), .o1(\s[24] ));
  norb02aa1n03x4               g145(.a(new_n235), .b(new_n236), .out0(new_n241));
  nano22aa1n06x5               g146(.a(new_n214), .b(new_n241), .c(new_n227), .out0(new_n242));
  aoai13aa1n02x5               g147(.a(new_n242), .b(new_n188), .c(new_n126), .d(new_n179), .o1(new_n243));
  oao003aa1n02x5               g148(.a(new_n183), .b(new_n192), .c(new_n185), .carry(new_n244));
  aoai13aa1n06x5               g149(.a(new_n227), .b(new_n209), .c(new_n213), .d(new_n244), .o1(new_n245));
  inv000aa1n02x5               g150(.a(new_n229), .o1(new_n246));
  inv030aa1n03x5               g151(.a(new_n241), .o1(new_n247));
  orn002aa1n03x5               g152(.a(\a[23] ), .b(\b[22] ), .o(new_n248));
  oaoi03aa1n06x5               g153(.a(\a[24] ), .b(\b[23] ), .c(new_n248), .o1(new_n249));
  inv040aa1n03x5               g154(.a(new_n249), .o1(new_n250));
  aoai13aa1n04x5               g155(.a(new_n250), .b(new_n247), .c(new_n245), .d(new_n246), .o1(new_n251));
  nanb02aa1n03x5               g156(.a(new_n251), .b(new_n243), .out0(new_n252));
  xorb03aa1n02x5               g157(.a(new_n252), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g158(.a(\b[24] ), .b(\a[25] ), .o1(new_n254));
  xorc02aa1n12x5               g159(.a(\a[25] ), .b(\b[24] ), .out0(new_n255));
  nor042aa1n03x5               g160(.a(\b[25] ), .b(\a[26] ), .o1(new_n256));
  nand02aa1d08x5               g161(.a(\b[25] ), .b(\a[26] ), .o1(new_n257));
  norb02aa1n03x5               g162(.a(new_n257), .b(new_n256), .out0(new_n258));
  inv040aa1n03x5               g163(.a(new_n258), .o1(new_n259));
  aoai13aa1n03x5               g164(.a(new_n259), .b(new_n254), .c(new_n252), .d(new_n255), .o1(new_n260));
  aoai13aa1n06x5               g165(.a(new_n255), .b(new_n251), .c(new_n184), .d(new_n242), .o1(new_n261));
  nona22aa1n03x5               g166(.a(new_n261), .b(new_n259), .c(new_n254), .out0(new_n262));
  nanp02aa1n03x5               g167(.a(new_n260), .b(new_n262), .o1(\s[26] ));
  norb02aa1n06x5               g168(.a(new_n255), .b(new_n259), .out0(new_n264));
  nand02aa1n02x5               g169(.a(new_n251), .b(new_n264), .o1(new_n265));
  oai012aa1n02x5               g170(.a(new_n257), .b(new_n256), .c(new_n254), .o1(new_n266));
  nano22aa1n03x7               g171(.a(new_n231), .b(new_n241), .c(new_n264), .out0(new_n267));
  aoai13aa1n06x5               g172(.a(new_n267), .b(new_n188), .c(new_n126), .d(new_n179), .o1(new_n268));
  nand23aa1n04x5               g173(.a(new_n265), .b(new_n268), .c(new_n266), .o1(new_n269));
  xorb03aa1n02x5               g174(.a(new_n269), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g175(.a(\b[26] ), .b(\a[27] ), .o1(new_n271));
  xorc02aa1n02x5               g176(.a(\a[27] ), .b(\b[26] ), .out0(new_n272));
  xnrc02aa1n02x5               g177(.a(\b[27] ), .b(\a[28] ), .out0(new_n273));
  aoai13aa1n03x5               g178(.a(new_n273), .b(new_n271), .c(new_n269), .d(new_n272), .o1(new_n274));
  aoai13aa1n06x5               g179(.a(new_n241), .b(new_n229), .c(new_n211), .d(new_n227), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n264), .o1(new_n276));
  aoai13aa1n04x5               g181(.a(new_n266), .b(new_n276), .c(new_n275), .d(new_n250), .o1(new_n277));
  aobi12aa1n09x5               g182(.a(new_n267), .b(new_n180), .c(new_n178), .out0(new_n278));
  oaih12aa1n02x5               g183(.a(new_n272), .b(new_n277), .c(new_n278), .o1(new_n279));
  nona22aa1n03x5               g184(.a(new_n279), .b(new_n273), .c(new_n271), .out0(new_n280));
  nanp02aa1n03x5               g185(.a(new_n274), .b(new_n280), .o1(\s[28] ));
  norb02aa1n02x5               g186(.a(new_n272), .b(new_n273), .out0(new_n282));
  oaih12aa1n02x5               g187(.a(new_n282), .b(new_n277), .c(new_n278), .o1(new_n283));
  inv000aa1n03x5               g188(.a(new_n271), .o1(new_n284));
  oaoi03aa1n02x5               g189(.a(\a[28] ), .b(\b[27] ), .c(new_n284), .o1(new_n285));
  xnrc02aa1n02x5               g190(.a(\b[28] ), .b(\a[29] ), .out0(new_n286));
  nona22aa1n03x5               g191(.a(new_n283), .b(new_n285), .c(new_n286), .out0(new_n287));
  aoai13aa1n03x5               g192(.a(new_n286), .b(new_n285), .c(new_n269), .d(new_n282), .o1(new_n288));
  nanp02aa1n03x5               g193(.a(new_n288), .b(new_n287), .o1(\s[29] ));
  xorb03aa1n02x5               g194(.a(new_n118), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g195(.a(new_n272), .b(new_n286), .c(new_n273), .out0(new_n291));
  oao003aa1n02x5               g196(.a(\a[28] ), .b(\b[27] ), .c(new_n284), .carry(new_n292));
  oaoi03aa1n02x5               g197(.a(\a[29] ), .b(\b[28] ), .c(new_n292), .o1(new_n293));
  tech160nm_fixorc02aa1n03p5x5 g198(.a(\a[30] ), .b(\b[29] ), .out0(new_n294));
  inv000aa1d42x5               g199(.a(new_n294), .o1(new_n295));
  aoai13aa1n03x5               g200(.a(new_n295), .b(new_n293), .c(new_n269), .d(new_n291), .o1(new_n296));
  oaih12aa1n02x5               g201(.a(new_n291), .b(new_n277), .c(new_n278), .o1(new_n297));
  nona22aa1n03x5               g202(.a(new_n297), .b(new_n293), .c(new_n295), .out0(new_n298));
  nanp02aa1n03x5               g203(.a(new_n296), .b(new_n298), .o1(\s[30] ));
  nanp02aa1n02x5               g204(.a(new_n293), .b(new_n294), .o1(new_n300));
  oai012aa1n02x5               g205(.a(new_n300), .b(\b[29] ), .c(\a[30] ), .o1(new_n301));
  nano23aa1n02x4               g206(.a(new_n286), .b(new_n273), .c(new_n294), .d(new_n272), .out0(new_n302));
  oaih12aa1n02x5               g207(.a(new_n302), .b(new_n277), .c(new_n278), .o1(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[30] ), .b(\a[31] ), .out0(new_n304));
  nona22aa1n03x5               g209(.a(new_n303), .b(new_n304), .c(new_n301), .out0(new_n305));
  aoai13aa1n03x5               g210(.a(new_n304), .b(new_n301), .c(new_n269), .d(new_n302), .o1(new_n306));
  nanp02aa1n03x5               g211(.a(new_n306), .b(new_n305), .o1(\s[31] ));
  xorb03aa1n02x5               g212(.a(new_n120), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  aoi012aa1n02x5               g213(.a(new_n114), .b(new_n120), .c(new_n115), .o1(new_n309));
  xnrc02aa1n02x5               g214(.a(new_n309), .b(new_n113), .out0(\s[4] ));
  xnbna2aa1n03x5               g215(.a(new_n124), .b(new_n121), .c(new_n122), .out0(\s[5] ));
  inv000aa1d42x5               g216(.a(\a[6] ), .o1(new_n312));
  nanp02aa1n02x5               g217(.a(new_n121), .b(new_n122), .o1(new_n313));
  oaoi03aa1n03x5               g218(.a(new_n105), .b(new_n106), .c(new_n313), .o1(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[5] ), .c(new_n312), .out0(\s[6] ));
  inv000aa1d42x5               g220(.a(\b[5] ), .o1(new_n316));
  norb02aa1n02x5               g221(.a(new_n103), .b(new_n102), .out0(new_n317));
  nanp02aa1n02x5               g222(.a(new_n314), .b(new_n123), .o1(new_n318));
  oai112aa1n03x5               g223(.a(new_n318), .b(new_n317), .c(new_n316), .d(new_n312), .o1(new_n319));
  oaoi13aa1n02x5               g224(.a(new_n317), .b(new_n318), .c(new_n312), .d(new_n316), .o1(new_n320));
  norb02aa1n02x5               g225(.a(new_n319), .b(new_n320), .out0(\s[7] ));
  norb02aa1n02x5               g226(.a(new_n100), .b(new_n101), .out0(new_n322));
  inv000aa1d42x5               g227(.a(new_n102), .o1(new_n323));
  xnbna2aa1n03x5               g228(.a(new_n322), .b(new_n319), .c(new_n323), .out0(\s[8] ));
  xorb03aa1n02x5               g229(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


