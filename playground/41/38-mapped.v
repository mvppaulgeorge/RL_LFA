// Benchmark "adder" written by ABC on Thu Jul 18 09:19:05 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n137, new_n138, new_n139,
    new_n140, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n161, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n213, new_n214, new_n215, new_n216,
    new_n218, new_n219, new_n220, new_n221, new_n222, new_n223, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n236, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n246, new_n247, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n282, new_n283, new_n284, new_n285, new_n286, new_n287, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n316, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n334, new_n336, new_n339, new_n341,
    new_n342, new_n343, new_n344, new_n346;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv040aa1d32x5               g001(.a(\a[10] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\b[9] ), .o1(new_n98));
  nand02aa1n08x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  nand02aa1d28x5               g004(.a(\b[9] ), .b(\a[10] ), .o1(new_n100));
  nand22aa1n12x5               g005(.a(new_n99), .b(new_n100), .o1(new_n101));
  inv000aa1d42x5               g006(.a(new_n101), .o1(new_n102));
  inv000aa1d42x5               g007(.a(\a[9] ), .o1(new_n103));
  nanb02aa1d36x5               g008(.a(\b[8] ), .b(new_n103), .out0(new_n104));
  tech160nm_finand02aa1n03p5x5 g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  norp02aa1n04x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nor042aa1n04x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  oai012aa1n04x7               g012(.a(new_n105), .b(new_n107), .c(new_n106), .o1(new_n108));
  norb02aa1n06x5               g013(.a(new_n105), .b(new_n106), .out0(new_n109));
  nand42aa1n03x5               g014(.a(\b[2] ), .b(\a[3] ), .o1(new_n110));
  norb02aa1n03x5               g015(.a(new_n110), .b(new_n107), .out0(new_n111));
  nor022aa1n06x5               g016(.a(\b[1] ), .b(\a[2] ), .o1(new_n112));
  aoi022aa1d18x5               g017(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n113));
  oai112aa1n06x5               g018(.a(new_n109), .b(new_n111), .c(new_n112), .d(new_n113), .o1(new_n114));
  nanp02aa1n03x5               g019(.a(new_n114), .b(new_n108), .o1(new_n115));
  nand42aa1n02x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nor002aa1n02x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  nanb02aa1n02x5               g022(.a(new_n117), .b(new_n116), .out0(new_n118));
  inv000aa1d42x5               g023(.a(\a[5] ), .o1(new_n119));
  nanb02aa1n12x5               g024(.a(\b[4] ), .b(new_n119), .out0(new_n120));
  nanp02aa1n02x5               g025(.a(\b[4] ), .b(\a[5] ), .o1(new_n121));
  nanp02aa1n02x5               g026(.a(new_n120), .b(new_n121), .o1(new_n122));
  norp02aa1n04x5               g027(.a(\b[7] ), .b(\a[8] ), .o1(new_n123));
  nand22aa1n04x5               g028(.a(\b[7] ), .b(\a[8] ), .o1(new_n124));
  nor022aa1n06x5               g029(.a(\b[6] ), .b(\a[7] ), .o1(new_n125));
  nand42aa1n02x5               g030(.a(\b[6] ), .b(\a[7] ), .o1(new_n126));
  nona23aa1n03x5               g031(.a(new_n126), .b(new_n124), .c(new_n123), .d(new_n125), .out0(new_n127));
  nor043aa1n03x5               g032(.a(new_n127), .b(new_n122), .c(new_n118), .o1(new_n128));
  oaoi03aa1n02x5               g033(.a(\a[6] ), .b(\b[5] ), .c(new_n120), .o1(new_n129));
  oai012aa1n02x5               g034(.a(new_n124), .b(new_n125), .c(new_n123), .o1(new_n130));
  oaib12aa1n06x5               g035(.a(new_n130), .b(new_n127), .c(new_n129), .out0(new_n131));
  nand42aa1n06x5               g036(.a(\b[8] ), .b(\a[9] ), .o1(new_n132));
  nand02aa1d16x5               g037(.a(new_n104), .b(new_n132), .o1(new_n133));
  inv000aa1d42x5               g038(.a(new_n133), .o1(new_n134));
  aoai13aa1n02x5               g039(.a(new_n134), .b(new_n131), .c(new_n115), .d(new_n128), .o1(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n102), .b(new_n135), .c(new_n104), .out0(\s[10] ));
  nanp03aa1n02x5               g041(.a(new_n135), .b(new_n102), .c(new_n104), .o1(new_n137));
  nor002aa1d32x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  nand42aa1n20x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  nanb02aa1n03x5               g044(.a(new_n138), .b(new_n139), .out0(new_n140));
  xnbna2aa1n03x5               g045(.a(new_n140), .b(new_n137), .c(new_n100), .out0(\s[11] ));
  inv040aa1n03x5               g046(.a(new_n138), .o1(new_n142));
  inv000aa1d42x5               g047(.a(new_n100), .o1(new_n143));
  nona22aa1n02x4               g048(.a(new_n137), .b(new_n140), .c(new_n143), .out0(new_n144));
  nor042aa1n06x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  nand42aa1n08x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nanb02aa1n03x5               g051(.a(new_n145), .b(new_n146), .out0(new_n147));
  xobna2aa1n03x5               g052(.a(new_n147), .b(new_n144), .c(new_n142), .out0(\s[12] ));
  nano23aa1d15x5               g053(.a(new_n138), .b(new_n145), .c(new_n146), .d(new_n139), .out0(new_n149));
  nona22aa1d30x5               g054(.a(new_n149), .b(new_n133), .c(new_n101), .out0(new_n150));
  inv000aa1d42x5               g055(.a(new_n150), .o1(new_n151));
  aoai13aa1n06x5               g056(.a(new_n151), .b(new_n131), .c(new_n115), .d(new_n128), .o1(new_n152));
  oai022aa1d24x5               g057(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n153));
  nano23aa1n03x5               g058(.a(new_n147), .b(new_n140), .c(new_n153), .d(new_n100), .out0(new_n154));
  oaoi03aa1n12x5               g059(.a(\a[12] ), .b(\b[11] ), .c(new_n142), .o1(new_n155));
  norp02aa1n02x5               g060(.a(new_n154), .b(new_n155), .o1(new_n156));
  nor002aa1d32x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  tech160nm_finand02aa1n03p5x5 g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  norb02aa1n02x7               g063(.a(new_n158), .b(new_n157), .out0(new_n159));
  xnbna2aa1n03x5               g064(.a(new_n159), .b(new_n152), .c(new_n156), .out0(\s[13] ));
  inv000aa1d42x5               g065(.a(new_n157), .o1(new_n161));
  nanb02aa1n03x5               g066(.a(new_n157), .b(new_n158), .out0(new_n162));
  aoai13aa1n02x5               g067(.a(new_n161), .b(new_n162), .c(new_n152), .d(new_n156), .o1(new_n163));
  xorb03aa1n02x5               g068(.a(new_n163), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nanp02aa1n02x5               g069(.a(new_n152), .b(new_n156), .o1(new_n165));
  inv000aa1d42x5               g070(.a(\a[14] ), .o1(new_n166));
  inv000aa1d42x5               g071(.a(\b[13] ), .o1(new_n167));
  nanp02aa1n02x5               g072(.a(new_n167), .b(new_n166), .o1(new_n168));
  nanp02aa1n02x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  nanp02aa1n02x5               g074(.a(new_n168), .b(new_n169), .o1(new_n170));
  nona22aa1n02x4               g075(.a(new_n165), .b(new_n162), .c(new_n170), .out0(new_n171));
  oaoi03aa1n02x5               g076(.a(new_n166), .b(new_n167), .c(new_n157), .o1(new_n172));
  nor002aa1d32x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  nand42aa1n08x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n174), .b(new_n173), .out0(new_n175));
  xnbna2aa1n03x5               g080(.a(new_n175), .b(new_n171), .c(new_n172), .out0(\s[15] ));
  inv000aa1d42x5               g081(.a(new_n173), .o1(new_n177));
  aoi112aa1n02x5               g082(.a(new_n170), .b(new_n162), .c(new_n152), .d(new_n156), .o1(new_n178));
  oaoi03aa1n02x5               g083(.a(\a[14] ), .b(\b[13] ), .c(new_n161), .o1(new_n179));
  oai012aa1n02x5               g084(.a(new_n175), .b(new_n178), .c(new_n179), .o1(new_n180));
  nor042aa1n04x5               g085(.a(\b[15] ), .b(\a[16] ), .o1(new_n181));
  nand42aa1n03x5               g086(.a(\b[15] ), .b(\a[16] ), .o1(new_n182));
  norb02aa1n02x5               g087(.a(new_n182), .b(new_n181), .out0(new_n183));
  aobi12aa1n02x5               g088(.a(new_n183), .b(new_n180), .c(new_n177), .out0(new_n184));
  nanp02aa1n02x5               g089(.a(new_n171), .b(new_n172), .o1(new_n185));
  aoi112aa1n02x5               g090(.a(new_n173), .b(new_n183), .c(new_n185), .d(new_n174), .o1(new_n186));
  norp02aa1n02x5               g091(.a(new_n184), .b(new_n186), .o1(\s[16] ));
  nano23aa1n03x5               g092(.a(new_n173), .b(new_n181), .c(new_n182), .d(new_n174), .out0(new_n188));
  nona22aa1n03x5               g093(.a(new_n188), .b(new_n162), .c(new_n170), .out0(new_n189));
  nor042aa1n06x5               g094(.a(new_n189), .b(new_n150), .o1(new_n190));
  aoai13aa1n06x5               g095(.a(new_n190), .b(new_n131), .c(new_n115), .d(new_n128), .o1(new_n191));
  nona23aa1n02x4               g096(.a(new_n182), .b(new_n174), .c(new_n173), .d(new_n181), .out0(new_n192));
  nano32aa1n02x4               g097(.a(new_n192), .b(new_n159), .c(new_n168), .d(new_n169), .out0(new_n193));
  oaoi03aa1n02x5               g098(.a(\a[16] ), .b(\b[15] ), .c(new_n177), .o1(new_n194));
  oabi12aa1n03x5               g099(.a(new_n194), .b(new_n192), .c(new_n172), .out0(new_n195));
  oaoi13aa1n06x5               g100(.a(new_n195), .b(new_n193), .c(new_n154), .d(new_n155), .o1(new_n196));
  tech160nm_fixorc02aa1n03p5x5 g101(.a(\a[17] ), .b(\b[16] ), .out0(new_n197));
  xnbna2aa1n03x5               g102(.a(new_n197), .b(new_n191), .c(new_n196), .out0(\s[17] ));
  inv040aa1d28x5               g103(.a(\a[17] ), .o1(new_n199));
  inv040aa1d32x5               g104(.a(\b[16] ), .o1(new_n200));
  nanp02aa1n02x5               g105(.a(new_n200), .b(new_n199), .o1(new_n201));
  nano23aa1n03x7               g106(.a(new_n123), .b(new_n125), .c(new_n126), .d(new_n124), .out0(new_n202));
  nona22aa1n03x5               g107(.a(new_n202), .b(new_n118), .c(new_n122), .out0(new_n203));
  oai022aa1n02x5               g108(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n204));
  aoi022aa1n03x5               g109(.a(new_n202), .b(new_n129), .c(new_n124), .d(new_n204), .o1(new_n205));
  aoai13aa1n06x5               g110(.a(new_n205), .b(new_n203), .c(new_n108), .d(new_n114), .o1(new_n206));
  inv000aa1d42x5               g111(.a(new_n153), .o1(new_n207));
  nona22aa1n02x4               g112(.a(new_n149), .b(new_n207), .c(new_n143), .out0(new_n208));
  inv000aa1d42x5               g113(.a(new_n155), .o1(new_n209));
  aoi012aa1n02x5               g114(.a(new_n194), .b(new_n188), .c(new_n179), .o1(new_n210));
  aoai13aa1n04x5               g115(.a(new_n210), .b(new_n189), .c(new_n208), .d(new_n209), .o1(new_n211));
  aoai13aa1n02x5               g116(.a(new_n197), .b(new_n211), .c(new_n206), .d(new_n190), .o1(new_n212));
  nor002aa1d32x5               g117(.a(\b[17] ), .b(\a[18] ), .o1(new_n213));
  nand42aa1d28x5               g118(.a(\b[17] ), .b(\a[18] ), .o1(new_n214));
  nanb02aa1n12x5               g119(.a(new_n213), .b(new_n214), .out0(new_n215));
  inv000aa1d42x5               g120(.a(new_n215), .o1(new_n216));
  xnbna2aa1n03x5               g121(.a(new_n216), .b(new_n212), .c(new_n201), .out0(\s[18] ));
  nand42aa1n06x5               g122(.a(new_n191), .b(new_n196), .o1(new_n218));
  nand23aa1n03x5               g123(.a(new_n218), .b(new_n197), .c(new_n216), .o1(new_n219));
  aoai13aa1n12x5               g124(.a(new_n214), .b(new_n213), .c(new_n199), .d(new_n200), .o1(new_n220));
  nor002aa1d32x5               g125(.a(\b[18] ), .b(\a[19] ), .o1(new_n221));
  nand02aa1n08x5               g126(.a(\b[18] ), .b(\a[19] ), .o1(new_n222));
  norb02aa1n02x5               g127(.a(new_n222), .b(new_n221), .out0(new_n223));
  xnbna2aa1n03x5               g128(.a(new_n223), .b(new_n219), .c(new_n220), .out0(\s[19] ));
  xnrc02aa1n02x5               g129(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv040aa1n08x5               g130(.a(new_n221), .o1(new_n226));
  tech160nm_fiaoi012aa1n05x5   g131(.a(new_n211), .b(new_n206), .c(new_n190), .o1(new_n227));
  nano22aa1n03x7               g132(.a(new_n227), .b(new_n197), .c(new_n216), .out0(new_n228));
  inv040aa1n08x5               g133(.a(new_n220), .o1(new_n229));
  oai012aa1n02x5               g134(.a(new_n223), .b(new_n228), .c(new_n229), .o1(new_n230));
  norp02aa1n12x5               g135(.a(\b[19] ), .b(\a[20] ), .o1(new_n231));
  nand02aa1n06x5               g136(.a(\b[19] ), .b(\a[20] ), .o1(new_n232));
  nanb02aa1n02x5               g137(.a(new_n231), .b(new_n232), .out0(new_n233));
  aoi012aa1n02x5               g138(.a(new_n233), .b(new_n230), .c(new_n226), .o1(new_n234));
  aobi12aa1n06x5               g139(.a(new_n223), .b(new_n219), .c(new_n220), .out0(new_n235));
  nano22aa1n03x7               g140(.a(new_n235), .b(new_n226), .c(new_n233), .out0(new_n236));
  norp02aa1n03x5               g141(.a(new_n234), .b(new_n236), .o1(\s[20] ));
  nona23aa1n09x5               g142(.a(new_n232), .b(new_n222), .c(new_n221), .d(new_n231), .out0(new_n238));
  norb03aa1n03x5               g143(.a(new_n197), .b(new_n238), .c(new_n215), .out0(new_n239));
  aoai13aa1n03x5               g144(.a(new_n239), .b(new_n211), .c(new_n206), .d(new_n190), .o1(new_n240));
  nano23aa1n03x7               g145(.a(new_n221), .b(new_n231), .c(new_n232), .d(new_n222), .out0(new_n241));
  oaoi03aa1n12x5               g146(.a(\a[20] ), .b(\b[19] ), .c(new_n226), .o1(new_n242));
  aoi012aa1n02x5               g147(.a(new_n242), .b(new_n241), .c(new_n229), .o1(new_n243));
  xnrc02aa1n12x5               g148(.a(\b[20] ), .b(\a[21] ), .out0(new_n244));
  xobna2aa1n03x5               g149(.a(new_n244), .b(new_n240), .c(new_n243), .out0(\s[21] ));
  orn002aa1n24x5               g150(.a(\a[21] ), .b(\b[20] ), .o(new_n246));
  aoai13aa1n03x5               g151(.a(new_n246), .b(new_n244), .c(new_n240), .d(new_n243), .o1(new_n247));
  xorb03aa1n02x5               g152(.a(new_n247), .b(\b[21] ), .c(\a[22] ), .out0(\s[22] ));
  inv040aa1n02x5               g153(.a(new_n242), .o1(new_n249));
  oai012aa1n12x5               g154(.a(new_n249), .b(new_n238), .c(new_n220), .o1(new_n250));
  xnrc02aa1n12x5               g155(.a(\b[21] ), .b(\a[22] ), .out0(new_n251));
  nor042aa1n09x5               g156(.a(new_n251), .b(new_n244), .o1(new_n252));
  oaoi03aa1n12x5               g157(.a(\a[22] ), .b(\b[21] ), .c(new_n246), .o1(new_n253));
  aoi012aa1d24x5               g158(.a(new_n253), .b(new_n250), .c(new_n252), .o1(new_n254));
  inv020aa1n03x5               g159(.a(new_n239), .o1(new_n255));
  nona32aa1n03x5               g160(.a(new_n218), .b(new_n251), .c(new_n244), .d(new_n255), .out0(new_n256));
  xnrc02aa1n12x5               g161(.a(\b[22] ), .b(\a[23] ), .out0(new_n257));
  xobna2aa1n03x5               g162(.a(new_n257), .b(new_n256), .c(new_n254), .out0(\s[23] ));
  orn002aa1n24x5               g163(.a(\a[23] ), .b(\b[22] ), .o(new_n259));
  inv000aa1d42x5               g164(.a(new_n254), .o1(new_n260));
  nano22aa1n03x7               g165(.a(new_n227), .b(new_n239), .c(new_n252), .out0(new_n261));
  oabi12aa1n06x5               g166(.a(new_n257), .b(new_n261), .c(new_n260), .out0(new_n262));
  xnrc02aa1n06x5               g167(.a(\b[23] ), .b(\a[24] ), .out0(new_n263));
  tech160nm_fiaoi012aa1n02p5x5 g168(.a(new_n263), .b(new_n262), .c(new_n259), .o1(new_n264));
  tech160nm_fiaoi012aa1n05x5   g169(.a(new_n257), .b(new_n256), .c(new_n254), .o1(new_n265));
  nano22aa1n03x7               g170(.a(new_n265), .b(new_n259), .c(new_n263), .out0(new_n266));
  nor002aa1n02x5               g171(.a(new_n264), .b(new_n266), .o1(\s[24] ));
  aoai13aa1n06x5               g172(.a(new_n252), .b(new_n242), .c(new_n241), .d(new_n229), .o1(new_n268));
  inv000aa1n02x5               g173(.a(new_n253), .o1(new_n269));
  nor042aa1n03x5               g174(.a(new_n263), .b(new_n257), .o1(new_n270));
  inv020aa1n02x5               g175(.a(new_n270), .o1(new_n271));
  oaoi03aa1n02x5               g176(.a(\a[24] ), .b(\b[23] ), .c(new_n259), .o1(new_n272));
  inv000aa1n03x5               g177(.a(new_n272), .o1(new_n273));
  aoai13aa1n12x5               g178(.a(new_n273), .b(new_n271), .c(new_n268), .d(new_n269), .o1(new_n274));
  inv000aa1d42x5               g179(.a(new_n274), .o1(new_n275));
  nanp02aa1n02x5               g180(.a(new_n270), .b(new_n252), .o1(new_n276));
  tech160nm_fixnrc02aa1n05x5   g181(.a(\b[24] ), .b(\a[25] ), .out0(new_n277));
  oaoi13aa1n03x5               g182(.a(new_n277), .b(new_n275), .c(new_n240), .d(new_n276), .o1(new_n278));
  nano32aa1n03x7               g183(.a(new_n227), .b(new_n270), .c(new_n239), .d(new_n252), .out0(new_n279));
  nano22aa1n02x4               g184(.a(new_n279), .b(new_n275), .c(new_n277), .out0(new_n280));
  norp02aa1n02x5               g185(.a(new_n278), .b(new_n280), .o1(\s[25] ));
  nor042aa1n03x5               g186(.a(\b[24] ), .b(\a[25] ), .o1(new_n282));
  inv000aa1d42x5               g187(.a(new_n282), .o1(new_n283));
  oabi12aa1n02x7               g188(.a(new_n277), .b(new_n279), .c(new_n274), .out0(new_n284));
  xnrc02aa1n02x5               g189(.a(\b[25] ), .b(\a[26] ), .out0(new_n285));
  tech160nm_fiaoi012aa1n02p5x5 g190(.a(new_n285), .b(new_n284), .c(new_n283), .o1(new_n286));
  nano22aa1n03x5               g191(.a(new_n278), .b(new_n283), .c(new_n285), .out0(new_n287));
  nor002aa1n02x5               g192(.a(new_n286), .b(new_n287), .o1(\s[26] ));
  xorc02aa1n12x5               g193(.a(\a[27] ), .b(\b[26] ), .out0(new_n289));
  nor042aa1n03x5               g194(.a(new_n285), .b(new_n277), .o1(new_n290));
  oao003aa1n02x5               g195(.a(\a[26] ), .b(\b[25] ), .c(new_n283), .carry(new_n291));
  aobi12aa1n12x5               g196(.a(new_n291), .b(new_n274), .c(new_n290), .out0(new_n292));
  inv000aa1n02x5               g197(.a(new_n290), .o1(new_n293));
  nona32aa1n09x5               g198(.a(new_n218), .b(new_n293), .c(new_n276), .d(new_n255), .out0(new_n294));
  xnbna2aa1n03x5               g199(.a(new_n289), .b(new_n292), .c(new_n294), .out0(\s[27] ));
  norp02aa1n02x5               g200(.a(\b[26] ), .b(\a[27] ), .o1(new_n296));
  inv040aa1n03x5               g201(.a(new_n296), .o1(new_n297));
  tech160nm_fiaoi012aa1n05x5   g202(.a(new_n255), .b(new_n191), .c(new_n196), .o1(new_n298));
  aoai13aa1n02x5               g203(.a(new_n270), .b(new_n253), .c(new_n250), .d(new_n252), .o1(new_n299));
  aoai13aa1n04x5               g204(.a(new_n291), .b(new_n293), .c(new_n299), .d(new_n273), .o1(new_n300));
  nano22aa1n02x4               g205(.a(new_n293), .b(new_n252), .c(new_n270), .out0(new_n301));
  aoai13aa1n03x5               g206(.a(new_n289), .b(new_n300), .c(new_n298), .d(new_n301), .o1(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[27] ), .b(\a[28] ), .out0(new_n303));
  tech160nm_fiaoi012aa1n02p5x5 g208(.a(new_n303), .b(new_n302), .c(new_n297), .o1(new_n304));
  aobi12aa1n03x5               g209(.a(new_n289), .b(new_n292), .c(new_n294), .out0(new_n305));
  nano22aa1n03x5               g210(.a(new_n305), .b(new_n297), .c(new_n303), .out0(new_n306));
  norp02aa1n03x5               g211(.a(new_n304), .b(new_n306), .o1(\s[28] ));
  xnrc02aa1n02x5               g212(.a(\b[28] ), .b(\a[29] ), .out0(new_n308));
  norb02aa1n02x5               g213(.a(new_n289), .b(new_n303), .out0(new_n309));
  aoai13aa1n03x5               g214(.a(new_n309), .b(new_n300), .c(new_n298), .d(new_n301), .o1(new_n310));
  oao003aa1n02x5               g215(.a(\a[28] ), .b(\b[27] ), .c(new_n297), .carry(new_n311));
  tech160nm_fiaoi012aa1n02p5x5 g216(.a(new_n308), .b(new_n310), .c(new_n311), .o1(new_n312));
  aobi12aa1n03x5               g217(.a(new_n309), .b(new_n292), .c(new_n294), .out0(new_n313));
  nano22aa1n03x5               g218(.a(new_n313), .b(new_n308), .c(new_n311), .out0(new_n314));
  norp02aa1n03x5               g219(.a(new_n312), .b(new_n314), .o1(\s[29] ));
  nanp02aa1n02x5               g220(.a(\b[0] ), .b(\a[1] ), .o1(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  xnrc02aa1n02x5               g222(.a(\b[29] ), .b(\a[30] ), .out0(new_n318));
  norb03aa1n02x5               g223(.a(new_n289), .b(new_n308), .c(new_n303), .out0(new_n319));
  aoai13aa1n03x5               g224(.a(new_n319), .b(new_n300), .c(new_n298), .d(new_n301), .o1(new_n320));
  oao003aa1n02x5               g225(.a(\a[29] ), .b(\b[28] ), .c(new_n311), .carry(new_n321));
  tech160nm_fiaoi012aa1n02p5x5 g226(.a(new_n318), .b(new_n320), .c(new_n321), .o1(new_n322));
  aobi12aa1n03x5               g227(.a(new_n319), .b(new_n292), .c(new_n294), .out0(new_n323));
  nano22aa1n03x5               g228(.a(new_n323), .b(new_n318), .c(new_n321), .out0(new_n324));
  norp02aa1n03x5               g229(.a(new_n322), .b(new_n324), .o1(\s[30] ));
  norb02aa1n03x4               g230(.a(new_n319), .b(new_n318), .out0(new_n326));
  aobi12aa1n03x5               g231(.a(new_n326), .b(new_n292), .c(new_n294), .out0(new_n327));
  oao003aa1n02x5               g232(.a(\a[30] ), .b(\b[29] ), .c(new_n321), .carry(new_n328));
  xnrc02aa1n02x5               g233(.a(\b[30] ), .b(\a[31] ), .out0(new_n329));
  nano22aa1n03x5               g234(.a(new_n327), .b(new_n328), .c(new_n329), .out0(new_n330));
  aoai13aa1n03x5               g235(.a(new_n326), .b(new_n300), .c(new_n298), .d(new_n301), .o1(new_n331));
  tech160nm_fiaoi012aa1n02p5x5 g236(.a(new_n329), .b(new_n331), .c(new_n328), .o1(new_n332));
  norp02aa1n03x5               g237(.a(new_n332), .b(new_n330), .o1(\s[31] ));
  norp02aa1n02x5               g238(.a(new_n113), .b(new_n112), .o1(new_n334));
  xnrb03aa1n02x5               g239(.a(new_n334), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi13aa1n02x5               g240(.a(new_n107), .b(new_n110), .c(new_n113), .d(new_n112), .o1(new_n336));
  xnrc02aa1n02x5               g241(.a(new_n336), .b(new_n109), .out0(\s[4] ));
  xobna2aa1n03x5               g242(.a(new_n122), .b(new_n114), .c(new_n108), .out0(\s[5] ));
  aoai13aa1n02x5               g243(.a(new_n120), .b(new_n122), .c(new_n114), .d(new_n108), .o1(new_n339));
  xorb03aa1n02x5               g244(.a(new_n339), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanb02aa1n02x5               g245(.a(new_n125), .b(new_n126), .out0(new_n341));
  inv000aa1d42x5               g246(.a(new_n341), .o1(new_n342));
  aoai13aa1n02x5               g247(.a(new_n342), .b(new_n117), .c(new_n339), .d(new_n116), .o1(new_n343));
  aoi112aa1n02x5               g248(.a(new_n342), .b(new_n117), .c(new_n339), .d(new_n116), .o1(new_n344));
  norb02aa1n02x5               g249(.a(new_n343), .b(new_n344), .out0(\s[7] ));
  oai012aa1n02x5               g250(.a(new_n343), .b(\b[6] ), .c(\a[7] ), .o1(new_n346));
  xorb03aa1n02x5               g251(.a(new_n346), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g252(.a(new_n206), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


