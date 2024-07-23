// Benchmark "adder" written by ABC on Thu Jul 18 02:42:28 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n169, new_n170,
    new_n172, new_n173, new_n174, new_n175, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n318, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n326, new_n328, new_n331, new_n332, new_n333,
    new_n334, new_n335, new_n337, new_n338;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nand42aa1d28x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  tech160nm_fioai012aa1n03p5x5 g004(.a(new_n99), .b(\b[2] ), .c(\a[3] ), .o1(new_n100));
  and002aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o(new_n101));
  inv040aa1d28x5               g006(.a(\a[4] ), .o1(new_n102));
  inv000aa1d42x5               g007(.a(\b[3] ), .o1(new_n103));
  aoi022aa1n06x5               g008(.a(new_n103), .b(new_n102), .c(\a[3] ), .d(\b[2] ), .o1(new_n104));
  nand22aa1n12x5               g009(.a(\b[0] ), .b(\a[1] ), .o1(new_n105));
  nor042aa1n06x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  nona22aa1n09x5               g011(.a(new_n99), .b(new_n106), .c(new_n105), .out0(new_n107));
  nona23aa1n09x5               g012(.a(new_n107), .b(new_n104), .c(new_n100), .d(new_n101), .out0(new_n108));
  aoi112aa1n02x7               g013(.a(\b[2] ), .b(\a[3] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n109));
  aoi012aa1n06x5               g014(.a(new_n109), .b(new_n102), .c(new_n103), .o1(new_n110));
  xnrc02aa1n12x5               g015(.a(\b[4] ), .b(\a[5] ), .out0(new_n111));
  nand42aa1n16x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nor002aa1d32x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nanb02aa1n12x5               g018(.a(new_n113), .b(new_n112), .out0(new_n114));
  xorc02aa1n12x5               g019(.a(\a[7] ), .b(\b[6] ), .out0(new_n115));
  nand42aa1n06x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  nor042aa1n03x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  norb02aa1n03x5               g022(.a(new_n116), .b(new_n117), .out0(new_n118));
  nona23aa1n09x5               g023(.a(new_n118), .b(new_n115), .c(new_n111), .d(new_n114), .out0(new_n119));
  nano22aa1n06x5               g024(.a(new_n117), .b(new_n112), .c(new_n116), .out0(new_n120));
  oaih22aa1n04x5               g025(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n121));
  nor002aa1d32x5               g026(.a(\b[6] ), .b(\a[7] ), .o1(new_n122));
  inv000aa1d42x5               g027(.a(new_n122), .o1(new_n123));
  oaoi03aa1n03x5               g028(.a(\a[8] ), .b(\b[7] ), .c(new_n123), .o1(new_n124));
  aoi013aa1n06x4               g029(.a(new_n124), .b(new_n120), .c(new_n115), .d(new_n121), .o1(new_n125));
  aoai13aa1n12x5               g030(.a(new_n125), .b(new_n119), .c(new_n108), .d(new_n110), .o1(new_n126));
  tech160nm_fixnrc02aa1n05x5   g031(.a(\b[8] ), .b(\a[9] ), .out0(new_n127));
  nanb02aa1n06x5               g032(.a(new_n127), .b(new_n126), .out0(new_n128));
  norp02aa1n24x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nand02aa1n16x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  norb02aa1n03x5               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  xnbna2aa1n03x5               g036(.a(new_n131), .b(new_n128), .c(new_n98), .out0(\s[10] ));
  inv040aa1d32x5               g037(.a(\a[11] ), .o1(new_n133));
  inv040aa1d28x5               g038(.a(\b[10] ), .o1(new_n134));
  nand42aa1n06x5               g039(.a(new_n134), .b(new_n133), .o1(new_n135));
  nanp02aa1n04x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  norp02aa1n02x5               g041(.a(new_n129), .b(new_n97), .o1(new_n137));
  nanp02aa1n03x5               g042(.a(new_n128), .b(new_n137), .o1(new_n138));
  aoi022aa1n02x5               g043(.a(new_n138), .b(new_n130), .c(new_n135), .d(new_n136), .o1(new_n139));
  nanp03aa1n02x5               g044(.a(new_n135), .b(new_n130), .c(new_n136), .o1(new_n140));
  nanb02aa1n02x5               g045(.a(new_n140), .b(new_n138), .out0(new_n141));
  norb02aa1n03x4               g046(.a(new_n141), .b(new_n139), .out0(\s[11] ));
  aoai13aa1n02x5               g047(.a(new_n135), .b(new_n140), .c(new_n128), .d(new_n137), .o1(new_n143));
  nor042aa1n03x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nand42aa1n04x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  norb02aa1n02x5               g050(.a(new_n145), .b(new_n144), .out0(new_n146));
  aboi22aa1n03x5               g051(.a(new_n144), .b(new_n145), .c(new_n133), .d(new_n134), .out0(new_n147));
  aoi022aa1n02x5               g052(.a(new_n141), .b(new_n147), .c(new_n143), .d(new_n146), .o1(\s[12] ));
  nanp02aa1n02x5               g053(.a(new_n108), .b(new_n110), .o1(new_n149));
  nanp02aa1n02x5               g054(.a(new_n115), .b(new_n118), .o1(new_n150));
  norp03aa1n06x5               g055(.a(new_n150), .b(new_n114), .c(new_n111), .o1(new_n151));
  inv000aa1n02x5               g056(.a(new_n125), .o1(new_n152));
  nano32aa1n03x7               g057(.a(new_n144), .b(new_n135), .c(new_n145), .d(new_n136), .out0(new_n153));
  nanb02aa1n06x5               g058(.a(new_n127), .b(new_n131), .out0(new_n154));
  norb02aa1n02x7               g059(.a(new_n153), .b(new_n154), .out0(new_n155));
  aoai13aa1n06x5               g060(.a(new_n155), .b(new_n152), .c(new_n149), .d(new_n151), .o1(new_n156));
  nano22aa1n03x7               g061(.a(new_n144), .b(new_n136), .c(new_n145), .out0(new_n157));
  oaih12aa1n02x5               g062(.a(new_n130), .b(\b[10] ), .c(\a[11] ), .o1(new_n158));
  oab012aa1n04x5               g063(.a(new_n158), .b(new_n97), .c(new_n129), .out0(new_n159));
  oaoi03aa1n12x5               g064(.a(\a[12] ), .b(\b[11] ), .c(new_n135), .o1(new_n160));
  tech160nm_fiao0012aa1n02p5x5 g065(.a(new_n160), .b(new_n159), .c(new_n157), .o(new_n161));
  nanb02aa1n06x5               g066(.a(new_n161), .b(new_n156), .out0(new_n162));
  xorb03aa1n02x5               g067(.a(new_n162), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n09x5               g068(.a(\b[12] ), .b(\a[13] ), .o1(new_n164));
  nand42aa1n08x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  nor042aa1n06x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  nand42aa1n16x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n167), .b(new_n166), .out0(new_n168));
  aoai13aa1n03x5               g073(.a(new_n168), .b(new_n164), .c(new_n162), .d(new_n165), .o1(new_n169));
  aoi112aa1n02x5               g074(.a(new_n164), .b(new_n168), .c(new_n162), .d(new_n165), .o1(new_n170));
  norb02aa1n03x4               g075(.a(new_n169), .b(new_n170), .out0(\s[14] ));
  nano23aa1d15x5               g076(.a(new_n164), .b(new_n166), .c(new_n167), .d(new_n165), .out0(new_n172));
  aoai13aa1n06x5               g077(.a(new_n172), .b(new_n161), .c(new_n126), .d(new_n155), .o1(new_n173));
  oai012aa1n18x5               g078(.a(new_n167), .b(new_n166), .c(new_n164), .o1(new_n174));
  xorc02aa1n12x5               g079(.a(\a[15] ), .b(\b[14] ), .out0(new_n175));
  xnbna2aa1n03x5               g080(.a(new_n175), .b(new_n173), .c(new_n174), .out0(\s[15] ));
  inv000aa1d42x5               g081(.a(new_n174), .o1(new_n177));
  aoai13aa1n02x5               g082(.a(new_n175), .b(new_n177), .c(new_n162), .d(new_n172), .o1(new_n178));
  nor042aa1n04x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  inv000aa1n03x5               g084(.a(new_n179), .o1(new_n180));
  inv000aa1d42x5               g085(.a(new_n175), .o1(new_n181));
  aoai13aa1n03x5               g086(.a(new_n180), .b(new_n181), .c(new_n173), .d(new_n174), .o1(new_n182));
  tech160nm_fixorc02aa1n05x5   g087(.a(\a[16] ), .b(\b[15] ), .out0(new_n183));
  norp02aa1n02x5               g088(.a(new_n183), .b(new_n179), .o1(new_n184));
  aoi022aa1n03x5               g089(.a(new_n182), .b(new_n183), .c(new_n178), .d(new_n184), .o1(\s[16] ));
  nand02aa1d08x5               g090(.a(new_n183), .b(new_n175), .o1(new_n186));
  nano23aa1d12x5               g091(.a(new_n154), .b(new_n186), .c(new_n153), .d(new_n172), .out0(new_n187));
  aoai13aa1n06x5               g092(.a(new_n187), .b(new_n152), .c(new_n149), .d(new_n151), .o1(new_n188));
  aoai13aa1n04x5               g093(.a(new_n172), .b(new_n160), .c(new_n159), .d(new_n157), .o1(new_n189));
  tech160nm_fiaoi012aa1n04x5   g094(.a(new_n186), .b(new_n189), .c(new_n174), .o1(new_n190));
  oaoi03aa1n02x5               g095(.a(\a[16] ), .b(\b[15] ), .c(new_n180), .o1(new_n191));
  nona22aa1n09x5               g096(.a(new_n188), .b(new_n190), .c(new_n191), .out0(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  tech160nm_finand02aa1n05x5   g098(.a(\b[16] ), .b(\a[17] ), .o1(new_n194));
  nor002aa1d24x5               g099(.a(\b[17] ), .b(\a[18] ), .o1(new_n195));
  nand02aa1d28x5               g100(.a(\b[17] ), .b(\a[18] ), .o1(new_n196));
  nano22aa1n02x4               g101(.a(new_n195), .b(new_n194), .c(new_n196), .out0(new_n197));
  nor042aa1d18x5               g102(.a(\b[16] ), .b(\a[17] ), .o1(new_n198));
  inv000aa1n02x5               g103(.a(new_n191), .o1(new_n199));
  nona23aa1n02x4               g104(.a(new_n188), .b(new_n199), .c(new_n190), .d(new_n198), .out0(new_n200));
  nanb02aa1n02x5               g105(.a(new_n195), .b(new_n196), .out0(new_n201));
  nanp02aa1n02x5               g106(.a(new_n200), .b(new_n194), .o1(new_n202));
  aoi022aa1n02x5               g107(.a(new_n202), .b(new_n201), .c(new_n197), .d(new_n200), .o1(\s[18] ));
  aoai13aa1n12x5               g108(.a(new_n199), .b(new_n186), .c(new_n189), .d(new_n174), .o1(new_n204));
  nano23aa1n06x5               g109(.a(new_n198), .b(new_n195), .c(new_n196), .d(new_n194), .out0(new_n205));
  aoai13aa1n06x5               g110(.a(new_n205), .b(new_n204), .c(new_n126), .d(new_n187), .o1(new_n206));
  aoi012aa1d24x5               g111(.a(new_n195), .b(new_n198), .c(new_n196), .o1(new_n207));
  xorc02aa1n12x5               g112(.a(\a[19] ), .b(\b[18] ), .out0(new_n208));
  xnbna2aa1n03x5               g113(.a(new_n208), .b(new_n206), .c(new_n207), .out0(\s[19] ));
  xnrc02aa1n02x5               g114(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g115(.a(new_n207), .o1(new_n211));
  aoai13aa1n03x5               g116(.a(new_n208), .b(new_n211), .c(new_n192), .d(new_n205), .o1(new_n212));
  inv000aa1d42x5               g117(.a(\a[19] ), .o1(new_n213));
  inv000aa1d42x5               g118(.a(\b[18] ), .o1(new_n214));
  nanp02aa1n02x5               g119(.a(new_n214), .b(new_n213), .o1(new_n215));
  inv000aa1d42x5               g120(.a(new_n208), .o1(new_n216));
  aoai13aa1n02x5               g121(.a(new_n215), .b(new_n216), .c(new_n206), .d(new_n207), .o1(new_n217));
  xorc02aa1n12x5               g122(.a(\a[20] ), .b(\b[19] ), .out0(new_n218));
  norb02aa1n02x5               g123(.a(new_n215), .b(new_n218), .out0(new_n219));
  aoi022aa1n03x5               g124(.a(new_n217), .b(new_n218), .c(new_n212), .d(new_n219), .o1(\s[20] ));
  nand23aa1n06x5               g125(.a(new_n205), .b(new_n208), .c(new_n218), .o1(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  aoai13aa1n06x5               g127(.a(new_n222), .b(new_n204), .c(new_n126), .d(new_n187), .o1(new_n223));
  inv000aa1d42x5               g128(.a(\b[19] ), .o1(new_n224));
  aboi22aa1d24x5               g129(.a(\a[20] ), .b(new_n224), .c(new_n213), .d(new_n214), .out0(new_n225));
  aoai13aa1n12x5               g130(.a(new_n225), .b(new_n207), .c(\a[19] ), .d(\b[18] ), .o1(new_n226));
  oaib12aa1n18x5               g131(.a(new_n226), .b(new_n224), .c(\a[20] ), .out0(new_n227));
  nor002aa1d32x5               g132(.a(\b[20] ), .b(\a[21] ), .o1(new_n228));
  nand02aa1n04x5               g133(.a(\b[20] ), .b(\a[21] ), .o1(new_n229));
  norb02aa1n06x5               g134(.a(new_n229), .b(new_n228), .out0(new_n230));
  xnbna2aa1n03x5               g135(.a(new_n230), .b(new_n223), .c(new_n227), .out0(\s[21] ));
  inv000aa1d42x5               g136(.a(new_n227), .o1(new_n232));
  aoai13aa1n06x5               g137(.a(new_n230), .b(new_n232), .c(new_n192), .d(new_n222), .o1(new_n233));
  inv000aa1n06x5               g138(.a(new_n228), .o1(new_n234));
  inv000aa1d42x5               g139(.a(new_n230), .o1(new_n235));
  aoai13aa1n02x5               g140(.a(new_n234), .b(new_n235), .c(new_n223), .d(new_n227), .o1(new_n236));
  norp02aa1n24x5               g141(.a(\b[21] ), .b(\a[22] ), .o1(new_n237));
  nanp02aa1n06x5               g142(.a(\b[21] ), .b(\a[22] ), .o1(new_n238));
  norb02aa1n02x5               g143(.a(new_n238), .b(new_n237), .out0(new_n239));
  aoib12aa1n02x5               g144(.a(new_n228), .b(new_n238), .c(new_n237), .out0(new_n240));
  aoi022aa1n03x5               g145(.a(new_n236), .b(new_n239), .c(new_n233), .d(new_n240), .o1(\s[22] ));
  xorc02aa1n02x5               g146(.a(\a[23] ), .b(\b[22] ), .out0(new_n242));
  nona23aa1d16x5               g147(.a(new_n238), .b(new_n229), .c(new_n228), .d(new_n237), .out0(new_n243));
  nano32aa1n02x4               g148(.a(new_n243), .b(new_n205), .c(new_n208), .d(new_n218), .out0(new_n244));
  aoai13aa1n06x5               g149(.a(new_n244), .b(new_n204), .c(new_n126), .d(new_n187), .o1(new_n245));
  oaoi03aa1n02x5               g150(.a(\a[22] ), .b(\b[21] ), .c(new_n234), .o1(new_n246));
  oab012aa1n06x5               g151(.a(new_n246), .b(new_n227), .c(new_n243), .out0(new_n247));
  xnbna2aa1n03x5               g152(.a(new_n242), .b(new_n245), .c(new_n247), .out0(\s[23] ));
  and002aa1n02x5               g153(.a(\b[22] ), .b(\a[23] ), .o(new_n249));
  aoi013aa1n02x4               g154(.a(new_n249), .b(new_n245), .c(new_n247), .d(new_n242), .o1(new_n250));
  xorc02aa1n02x5               g155(.a(\a[24] ), .b(\b[23] ), .out0(new_n251));
  inv000aa1d42x5               g156(.a(new_n251), .o1(new_n252));
  nand03aa1n02x5               g157(.a(new_n245), .b(new_n242), .c(new_n247), .o1(new_n253));
  nona22aa1n02x5               g158(.a(new_n253), .b(new_n251), .c(new_n249), .out0(new_n254));
  oaih12aa1n02x5               g159(.a(new_n254), .b(new_n250), .c(new_n252), .o1(\s[24] ));
  inv000aa1d42x5               g160(.a(new_n243), .o1(new_n256));
  inv000aa1d42x5               g161(.a(\a[23] ), .o1(new_n257));
  inv040aa1d32x5               g162(.a(\a[24] ), .o1(new_n258));
  xroi22aa1d06x4               g163(.a(new_n257), .b(\b[22] ), .c(new_n258), .d(\b[23] ), .out0(new_n259));
  nano22aa1n03x7               g164(.a(new_n221), .b(new_n259), .c(new_n256), .out0(new_n260));
  aoai13aa1n06x5               g165(.a(new_n260), .b(new_n204), .c(new_n126), .d(new_n187), .o1(new_n261));
  nanp02aa1n02x5               g166(.a(\b[23] ), .b(\a[24] ), .o1(new_n262));
  oai022aa1n02x5               g167(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n263));
  aoi022aa1n06x5               g168(.a(new_n259), .b(new_n246), .c(new_n262), .d(new_n263), .o1(new_n264));
  and002aa1n02x5               g169(.a(\b[19] ), .b(\a[20] ), .o(new_n265));
  nona23aa1n09x5               g170(.a(new_n226), .b(new_n259), .c(new_n243), .d(new_n265), .out0(new_n266));
  nand22aa1n03x5               g171(.a(new_n266), .b(new_n264), .o1(new_n267));
  inv000aa1n02x5               g172(.a(new_n267), .o1(new_n268));
  xorc02aa1n12x5               g173(.a(\a[25] ), .b(\b[24] ), .out0(new_n269));
  xnbna2aa1n03x5               g174(.a(new_n269), .b(new_n261), .c(new_n268), .out0(\s[25] ));
  aoai13aa1n03x5               g175(.a(new_n269), .b(new_n267), .c(new_n192), .d(new_n260), .o1(new_n271));
  nor042aa1n03x5               g176(.a(\b[24] ), .b(\a[25] ), .o1(new_n272));
  inv000aa1d42x5               g177(.a(new_n272), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n269), .o1(new_n274));
  aoai13aa1n02x7               g179(.a(new_n273), .b(new_n274), .c(new_n261), .d(new_n268), .o1(new_n275));
  tech160nm_fixorc02aa1n02p5x5 g180(.a(\a[26] ), .b(\b[25] ), .out0(new_n276));
  norp02aa1n02x5               g181(.a(new_n276), .b(new_n272), .o1(new_n277));
  aoi022aa1n03x5               g182(.a(new_n275), .b(new_n276), .c(new_n271), .d(new_n277), .o1(\s[26] ));
  norp02aa1n02x5               g183(.a(\b[26] ), .b(\a[27] ), .o1(new_n279));
  nanp02aa1n02x5               g184(.a(\b[26] ), .b(\a[27] ), .o1(new_n280));
  nanb02aa1n06x5               g185(.a(new_n279), .b(new_n280), .out0(new_n281));
  and002aa1n02x5               g186(.a(new_n276), .b(new_n269), .o(new_n282));
  nano32aa1n03x7               g187(.a(new_n221), .b(new_n282), .c(new_n256), .d(new_n259), .out0(new_n283));
  aoai13aa1n06x5               g188(.a(new_n283), .b(new_n204), .c(new_n126), .d(new_n187), .o1(new_n284));
  inv000aa1n02x5               g189(.a(new_n282), .o1(new_n285));
  oao003aa1n02x5               g190(.a(\a[26] ), .b(\b[25] ), .c(new_n273), .carry(new_n286));
  aoai13aa1n06x5               g191(.a(new_n286), .b(new_n285), .c(new_n266), .d(new_n264), .o1(new_n287));
  inv000aa1n02x5               g192(.a(new_n287), .o1(new_n288));
  xobna2aa1n03x5               g193(.a(new_n281), .b(new_n284), .c(new_n288), .out0(\s[27] ));
  inv000aa1d42x5               g194(.a(\a[28] ), .o1(new_n290));
  inv000aa1d42x5               g195(.a(\b[27] ), .o1(new_n291));
  aob012aa1n02x5               g196(.a(new_n280), .b(\b[27] ), .c(\a[28] ), .out0(new_n292));
  aoi012aa1n02x5               g197(.a(new_n292), .b(new_n290), .c(new_n291), .o1(new_n293));
  nona22aa1n03x5               g198(.a(new_n284), .b(new_n287), .c(new_n279), .out0(new_n294));
  xnrc02aa1n12x5               g199(.a(\b[27] ), .b(\a[28] ), .out0(new_n295));
  nanp02aa1n03x5               g200(.a(new_n294), .b(new_n280), .o1(new_n296));
  aoi022aa1n02x7               g201(.a(new_n296), .b(new_n295), .c(new_n293), .d(new_n294), .o1(\s[28] ));
  nor042aa1n04x5               g202(.a(new_n295), .b(new_n281), .o1(new_n298));
  aoai13aa1n02x7               g203(.a(new_n298), .b(new_n287), .c(new_n192), .d(new_n283), .o1(new_n299));
  inv040aa1n03x5               g204(.a(new_n298), .o1(new_n300));
  oaoi03aa1n02x5               g205(.a(new_n290), .b(new_n291), .c(new_n279), .o1(new_n301));
  aoai13aa1n03x5               g206(.a(new_n301), .b(new_n300), .c(new_n284), .d(new_n288), .o1(new_n302));
  xorc02aa1n02x5               g207(.a(\a[29] ), .b(\b[28] ), .out0(new_n303));
  norb02aa1n02x5               g208(.a(new_n301), .b(new_n303), .out0(new_n304));
  aoi022aa1n03x5               g209(.a(new_n302), .b(new_n303), .c(new_n299), .d(new_n304), .o1(\s[29] ));
  xorb03aa1n02x5               g210(.a(new_n105), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1d15x5               g211(.a(new_n303), .b(new_n281), .c(new_n295), .out0(new_n307));
  aoai13aa1n03x5               g212(.a(new_n307), .b(new_n287), .c(new_n192), .d(new_n283), .o1(new_n308));
  inv000aa1d42x5               g213(.a(new_n307), .o1(new_n309));
  oaoi03aa1n02x5               g214(.a(\a[29] ), .b(\b[28] ), .c(new_n301), .o1(new_n310));
  inv000aa1n03x5               g215(.a(new_n310), .o1(new_n311));
  aoai13aa1n03x5               g216(.a(new_n311), .b(new_n309), .c(new_n284), .d(new_n288), .o1(new_n312));
  xorc02aa1n02x5               g217(.a(\a[30] ), .b(\b[29] ), .out0(new_n313));
  aoi012aa1n02x5               g218(.a(new_n301), .b(\a[29] ), .c(\b[28] ), .o1(new_n314));
  oabi12aa1n02x5               g219(.a(new_n313), .b(\a[29] ), .c(\b[28] ), .out0(new_n315));
  norp02aa1n02x5               g220(.a(new_n315), .b(new_n314), .o1(new_n316));
  aoi022aa1n03x5               g221(.a(new_n312), .b(new_n313), .c(new_n308), .d(new_n316), .o1(\s[30] ));
  nano22aa1n12x5               g222(.a(new_n300), .b(new_n303), .c(new_n313), .out0(new_n318));
  aoai13aa1n02x7               g223(.a(new_n318), .b(new_n287), .c(new_n192), .d(new_n283), .o1(new_n319));
  inv000aa1n02x5               g224(.a(new_n318), .o1(new_n320));
  oao003aa1n02x5               g225(.a(\a[30] ), .b(\b[29] ), .c(new_n311), .carry(new_n321));
  aoai13aa1n02x7               g226(.a(new_n321), .b(new_n320), .c(new_n284), .d(new_n288), .o1(new_n322));
  xorc02aa1n02x5               g227(.a(\a[31] ), .b(\b[30] ), .out0(new_n323));
  norb02aa1n02x5               g228(.a(new_n321), .b(new_n323), .out0(new_n324));
  aoi022aa1n03x5               g229(.a(new_n322), .b(new_n323), .c(new_n319), .d(new_n324), .o1(\s[31] ));
  oai012aa1n02x5               g230(.a(new_n99), .b(new_n106), .c(new_n105), .o1(new_n326));
  xnrb03aa1n02x5               g231(.a(new_n326), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g232(.a(\a[3] ), .b(\b[2] ), .c(new_n326), .o1(new_n328));
  xorb03aa1n02x5               g233(.a(new_n328), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xobna2aa1n03x5               g234(.a(new_n111), .b(new_n108), .c(new_n110), .out0(\s[5] ));
  inv000aa1d42x5               g235(.a(new_n113), .o1(new_n331));
  nanb03aa1n02x5               g236(.a(new_n111), .b(new_n108), .c(new_n110), .out0(new_n332));
  and002aa1n02x5               g237(.a(\b[4] ), .b(\a[5] ), .o(new_n333));
  aboi22aa1n03x5               g238(.a(new_n333), .b(new_n332), .c(new_n112), .d(new_n331), .out0(new_n334));
  aoi022aa1n02x5               g239(.a(\b[5] ), .b(\a[6] ), .c(\a[5] ), .d(\b[4] ), .o1(new_n335));
  aoi013aa1n02x4               g240(.a(new_n334), .b(new_n332), .c(new_n331), .d(new_n335), .o1(\s[6] ));
  aoai13aa1n03x5               g241(.a(new_n115), .b(new_n113), .c(new_n332), .d(new_n335), .o1(new_n337));
  aoi112aa1n02x5               g242(.a(new_n113), .b(new_n115), .c(new_n332), .d(new_n335), .o1(new_n338));
  norb02aa1n02x5               g243(.a(new_n337), .b(new_n338), .out0(\s[7] ));
  xnbna2aa1n03x5               g244(.a(new_n118), .b(new_n337), .c(new_n123), .out0(\s[8] ));
  xorb03aa1n02x5               g245(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


