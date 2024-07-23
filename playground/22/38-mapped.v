// Benchmark "adder" written by ABC on Wed Jul 17 23:34:58 2024

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
    new_n125, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n134, new_n135, new_n136, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n154, new_n155, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n183, new_n184, new_n185, new_n186, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n214,
    new_n215, new_n216, new_n217, new_n218, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n230,
    new_n231, new_n232, new_n233, new_n234, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n252, new_n253,
    new_n254, new_n255, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n309,
    new_n311, new_n312, new_n314, new_n316, new_n318, new_n320, new_n321;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1d18x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand42aa1d28x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  nanb02aa1n02x5               g003(.a(new_n97), .b(new_n98), .out0(new_n99));
  nor042aa1d18x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  xnrc02aa1n02x5               g005(.a(\b[2] ), .b(\a[3] ), .out0(new_n101));
  inv000aa1d42x5               g006(.a(\a[2] ), .o1(new_n102));
  inv000aa1d42x5               g007(.a(\b[1] ), .o1(new_n103));
  nand42aa1n04x5               g008(.a(\b[0] ), .b(\a[1] ), .o1(new_n104));
  oao003aa1n02x5               g009(.a(new_n102), .b(new_n103), .c(new_n104), .carry(new_n105));
  nanb02aa1n02x5               g010(.a(new_n101), .b(new_n105), .out0(new_n106));
  oa0022aa1n09x5               g011(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n107));
  nor002aa1n02x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  aoi122aa1n06x5               g013(.a(new_n108), .b(\b[6] ), .c(\a[7] ), .d(\b[4] ), .e(\a[5] ), .o1(new_n109));
  tech160nm_fixorc02aa1n03p5x5 g014(.a(\a[8] ), .b(\b[7] ), .out0(new_n110));
  nor022aa1n04x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nand42aa1d28x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  oai012aa1n06x5               g017(.a(new_n112), .b(\b[4] ), .c(\a[5] ), .o1(new_n113));
  aoi112aa1n03x5               g018(.a(new_n113), .b(new_n111), .c(\a[6] ), .d(\b[5] ), .o1(new_n114));
  nanp03aa1n02x5               g019(.a(new_n114), .b(new_n109), .c(new_n110), .o1(new_n115));
  norp02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  aoi022aa1n02x5               g021(.a(\b[6] ), .b(\a[7] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n117));
  oaih12aa1n02x5               g022(.a(new_n117), .b(new_n116), .c(new_n108), .o1(new_n118));
  oab012aa1n03x5               g023(.a(new_n111), .b(\a[8] ), .c(\b[7] ), .out0(new_n119));
  aoi022aa1n06x5               g024(.a(new_n118), .b(new_n119), .c(\b[7] ), .d(\a[8] ), .o1(new_n120));
  inv000aa1n03x5               g025(.a(new_n120), .o1(new_n121));
  aoai13aa1n06x5               g026(.a(new_n121), .b(new_n115), .c(new_n106), .d(new_n107), .o1(new_n122));
  nand42aa1n08x5               g027(.a(\b[8] ), .b(\a[9] ), .o1(new_n123));
  aoai13aa1n02x5               g028(.a(new_n99), .b(new_n100), .c(new_n122), .d(new_n123), .o1(new_n124));
  aoi112aa1n03x5               g029(.a(new_n99), .b(new_n100), .c(new_n122), .d(new_n123), .o1(new_n125));
  nanb02aa1n02x5               g030(.a(new_n125), .b(new_n124), .out0(\s[10] ));
  nor002aa1d32x5               g031(.a(\b[10] ), .b(\a[11] ), .o1(new_n127));
  inv000aa1d42x5               g032(.a(new_n127), .o1(new_n128));
  nand42aa1n20x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  aboi22aa1n03x5               g034(.a(new_n125), .b(new_n98), .c(new_n129), .d(new_n128), .out0(new_n130));
  norb02aa1n02x5               g035(.a(new_n129), .b(new_n127), .out0(new_n131));
  nano22aa1n03x7               g036(.a(new_n125), .b(new_n98), .c(new_n131), .out0(new_n132));
  norp02aa1n02x5               g037(.a(new_n130), .b(new_n132), .o1(\s[11] ));
  xnrc02aa1n12x5               g038(.a(\b[11] ), .b(\a[12] ), .out0(new_n134));
  oab012aa1n03x5               g039(.a(new_n134), .b(new_n132), .c(new_n127), .out0(new_n135));
  nano22aa1n02x4               g040(.a(new_n132), .b(new_n128), .c(new_n134), .out0(new_n136));
  norp02aa1n02x5               g041(.a(new_n135), .b(new_n136), .o1(\s[12] ));
  tech160nm_fioaoi03aa1n03p5x5 g042(.a(new_n102), .b(new_n103), .c(new_n104), .o1(new_n138));
  oai012aa1n04x7               g043(.a(new_n107), .b(new_n138), .c(new_n101), .o1(new_n139));
  and003aa1n06x5               g044(.a(new_n114), .b(new_n110), .c(new_n109), .o(new_n140));
  norb03aa1d15x5               g045(.a(new_n98), .b(new_n97), .c(new_n100), .out0(new_n141));
  nano22aa1d15x5               g046(.a(new_n127), .b(new_n123), .c(new_n129), .out0(new_n142));
  nanb03aa1d24x5               g047(.a(new_n134), .b(new_n142), .c(new_n141), .out0(new_n143));
  inv000aa1d42x5               g048(.a(new_n143), .o1(new_n144));
  aoai13aa1n02x5               g049(.a(new_n144), .b(new_n120), .c(new_n140), .d(new_n139), .o1(new_n145));
  oai112aa1n06x5               g050(.a(new_n129), .b(new_n98), .c(new_n100), .d(new_n97), .o1(new_n146));
  oab012aa1n04x5               g051(.a(new_n127), .b(\a[12] ), .c(\b[11] ), .out0(new_n147));
  aoi022aa1d18x5               g052(.a(new_n146), .b(new_n147), .c(\b[11] ), .d(\a[12] ), .o1(new_n148));
  inv000aa1d42x5               g053(.a(new_n148), .o1(new_n149));
  nanp02aa1n02x5               g054(.a(new_n145), .b(new_n149), .o1(new_n150));
  xorb03aa1n02x5               g055(.a(new_n150), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv040aa1d32x5               g056(.a(\a[14] ), .o1(new_n152));
  nor002aa1d32x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nand42aa1n04x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  aoi012aa1n02x5               g059(.a(new_n153), .b(new_n150), .c(new_n154), .o1(new_n155));
  xorb03aa1n02x5               g060(.a(new_n155), .b(\b[13] ), .c(new_n152), .out0(\s[14] ));
  inv040aa1n02x5               g061(.a(new_n153), .o1(new_n157));
  xnrc02aa1n12x5               g062(.a(\b[13] ), .b(\a[14] ), .out0(new_n158));
  nano22aa1d24x5               g063(.a(new_n158), .b(new_n157), .c(new_n154), .out0(new_n159));
  inv000aa1d42x5               g064(.a(new_n159), .o1(new_n160));
  inv040aa1d32x5               g065(.a(\b[13] ), .o1(new_n161));
  oaoi03aa1n12x5               g066(.a(new_n152), .b(new_n161), .c(new_n153), .o1(new_n162));
  aoai13aa1n04x5               g067(.a(new_n162), .b(new_n160), .c(new_n145), .d(new_n149), .o1(new_n163));
  xorb03aa1n02x5               g068(.a(new_n163), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n06x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  nand42aa1n06x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  nor042aa1n06x5               g071(.a(\b[15] ), .b(\a[16] ), .o1(new_n167));
  nand42aa1n04x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n168), .b(new_n167), .out0(new_n169));
  aoi112aa1n02x5               g074(.a(new_n165), .b(new_n169), .c(new_n163), .d(new_n166), .o1(new_n170));
  aoai13aa1n02x5               g075(.a(new_n169), .b(new_n165), .c(new_n163), .d(new_n166), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n171), .b(new_n170), .out0(\s[16] ));
  nano23aa1d15x5               g077(.a(new_n165), .b(new_n167), .c(new_n168), .d(new_n166), .out0(new_n173));
  nano22aa1d15x5               g078(.a(new_n143), .b(new_n159), .c(new_n173), .out0(new_n174));
  aoai13aa1n12x5               g079(.a(new_n174), .b(new_n120), .c(new_n140), .d(new_n139), .o1(new_n175));
  inv040aa1n02x5               g080(.a(new_n165), .o1(new_n176));
  inv000aa1d42x5               g081(.a(new_n167), .o1(new_n177));
  nand42aa1n03x5               g082(.a(new_n168), .b(new_n166), .o1(new_n178));
  aoai13aa1n12x5               g083(.a(new_n177), .b(new_n178), .c(new_n162), .d(new_n176), .o1(new_n179));
  aoi013aa1n09x5               g084(.a(new_n179), .b(new_n148), .c(new_n159), .d(new_n173), .o1(new_n180));
  nand02aa1d08x5               g085(.a(new_n175), .b(new_n180), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g087(.a(\a[18] ), .o1(new_n183));
  inv000aa1d42x5               g088(.a(\a[17] ), .o1(new_n184));
  inv000aa1d42x5               g089(.a(\b[16] ), .o1(new_n185));
  oaoi03aa1n02x5               g090(.a(new_n184), .b(new_n185), .c(new_n181), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[17] ), .c(new_n183), .out0(\s[18] ));
  xroi22aa1d06x4               g092(.a(new_n184), .b(\b[16] ), .c(new_n183), .d(\b[17] ), .out0(new_n188));
  oai022aa1d24x5               g093(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n189));
  oaib12aa1n18x5               g094(.a(new_n189), .b(new_n183), .c(\b[17] ), .out0(new_n190));
  inv000aa1n09x5               g095(.a(new_n190), .o1(new_n191));
  nor042aa1d18x5               g096(.a(\b[18] ), .b(\a[19] ), .o1(new_n192));
  nand22aa1n09x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  norb02aa1n02x5               g098(.a(new_n193), .b(new_n192), .out0(new_n194));
  aoai13aa1n06x5               g099(.a(new_n194), .b(new_n191), .c(new_n181), .d(new_n188), .o1(new_n195));
  aoi112aa1n02x5               g100(.a(new_n194), .b(new_n191), .c(new_n181), .d(new_n188), .o1(new_n196));
  norb02aa1n02x5               g101(.a(new_n195), .b(new_n196), .out0(\s[19] ));
  xnrc02aa1n02x5               g102(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor022aa1n16x5               g103(.a(\b[19] ), .b(\a[20] ), .o1(new_n199));
  nand02aa1d28x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  norb02aa1n02x5               g105(.a(new_n200), .b(new_n199), .out0(new_n201));
  nona22aa1n06x5               g106(.a(new_n195), .b(new_n201), .c(new_n192), .out0(new_n202));
  orn002aa1n02x5               g107(.a(\a[19] ), .b(\b[18] ), .o(new_n203));
  aobi12aa1n06x5               g108(.a(new_n201), .b(new_n195), .c(new_n203), .out0(new_n204));
  norb02aa1n03x4               g109(.a(new_n202), .b(new_n204), .out0(\s[20] ));
  nano23aa1n06x5               g110(.a(new_n192), .b(new_n199), .c(new_n200), .d(new_n193), .out0(new_n206));
  nanp02aa1n02x5               g111(.a(new_n188), .b(new_n206), .o1(new_n207));
  nona23aa1n12x5               g112(.a(new_n200), .b(new_n193), .c(new_n192), .d(new_n199), .out0(new_n208));
  aoi012aa1n06x5               g113(.a(new_n199), .b(new_n192), .c(new_n200), .o1(new_n209));
  oai012aa1d24x5               g114(.a(new_n209), .b(new_n208), .c(new_n190), .o1(new_n210));
  inv000aa1d42x5               g115(.a(new_n210), .o1(new_n211));
  aoai13aa1n06x5               g116(.a(new_n211), .b(new_n207), .c(new_n175), .d(new_n180), .o1(new_n212));
  xorb03aa1n02x5               g117(.a(new_n212), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1n03x5               g118(.a(\b[20] ), .b(\a[21] ), .o1(new_n214));
  xorc02aa1n02x5               g119(.a(\a[21] ), .b(\b[20] ), .out0(new_n215));
  xorc02aa1n02x5               g120(.a(\a[22] ), .b(\b[21] ), .out0(new_n216));
  aoi112aa1n03x5               g121(.a(new_n214), .b(new_n216), .c(new_n212), .d(new_n215), .o1(new_n217));
  aoai13aa1n03x5               g122(.a(new_n216), .b(new_n214), .c(new_n212), .d(new_n215), .o1(new_n218));
  norb02aa1n03x4               g123(.a(new_n218), .b(new_n217), .out0(\s[22] ));
  inv000aa1d42x5               g124(.a(\a[21] ), .o1(new_n220));
  inv040aa1d32x5               g125(.a(\a[22] ), .o1(new_n221));
  xroi22aa1d06x4               g126(.a(new_n220), .b(\b[20] ), .c(new_n221), .d(\b[21] ), .out0(new_n222));
  nanp03aa1n02x5               g127(.a(new_n222), .b(new_n188), .c(new_n206), .o1(new_n223));
  inv000aa1d42x5               g128(.a(\b[21] ), .o1(new_n224));
  oaoi03aa1n12x5               g129(.a(new_n221), .b(new_n224), .c(new_n214), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  aoi012aa1n02x5               g131(.a(new_n226), .b(new_n210), .c(new_n222), .o1(new_n227));
  aoai13aa1n06x5               g132(.a(new_n227), .b(new_n223), .c(new_n175), .d(new_n180), .o1(new_n228));
  xorb03aa1n02x5               g133(.a(new_n228), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g134(.a(\b[22] ), .b(\a[23] ), .o1(new_n230));
  tech160nm_fixorc02aa1n05x5   g135(.a(\a[23] ), .b(\b[22] ), .out0(new_n231));
  tech160nm_fixorc02aa1n05x5   g136(.a(\a[24] ), .b(\b[23] ), .out0(new_n232));
  aoi112aa1n02x5               g137(.a(new_n230), .b(new_n232), .c(new_n228), .d(new_n231), .o1(new_n233));
  aoai13aa1n03x5               g138(.a(new_n232), .b(new_n230), .c(new_n228), .d(new_n231), .o1(new_n234));
  norb02aa1n03x4               g139(.a(new_n234), .b(new_n233), .out0(\s[24] ));
  nanp02aa1n02x5               g140(.a(new_n159), .b(new_n173), .o1(new_n236));
  inv000aa1d42x5               g141(.a(new_n179), .o1(new_n237));
  oaib12aa1n09x5               g142(.a(new_n237), .b(new_n236), .c(new_n148), .out0(new_n238));
  and002aa1n06x5               g143(.a(new_n232), .b(new_n231), .o(new_n239));
  inv000aa1n02x5               g144(.a(new_n239), .o1(new_n240));
  nano32aa1n02x4               g145(.a(new_n240), .b(new_n222), .c(new_n188), .d(new_n206), .out0(new_n241));
  aoai13aa1n02x5               g146(.a(new_n241), .b(new_n238), .c(new_n122), .d(new_n174), .o1(new_n242));
  inv020aa1n03x5               g147(.a(new_n209), .o1(new_n243));
  aoai13aa1n06x5               g148(.a(new_n222), .b(new_n243), .c(new_n206), .d(new_n191), .o1(new_n244));
  aoi112aa1n02x5               g149(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n245));
  oab012aa1n02x4               g150(.a(new_n245), .b(\a[24] ), .c(\b[23] ), .out0(new_n246));
  aoai13aa1n12x5               g151(.a(new_n246), .b(new_n240), .c(new_n244), .d(new_n225), .o1(new_n247));
  inv000aa1d42x5               g152(.a(new_n247), .o1(new_n248));
  xnrc02aa1n12x5               g153(.a(\b[24] ), .b(\a[25] ), .out0(new_n249));
  inv000aa1d42x5               g154(.a(new_n249), .o1(new_n250));
  xnbna2aa1n03x5               g155(.a(new_n250), .b(new_n242), .c(new_n248), .out0(\s[25] ));
  aoai13aa1n06x5               g156(.a(new_n250), .b(new_n247), .c(new_n181), .d(new_n241), .o1(new_n252));
  xnrc02aa1n06x5               g157(.a(\b[25] ), .b(\a[26] ), .out0(new_n253));
  oai112aa1n03x5               g158(.a(new_n252), .b(new_n253), .c(\b[24] ), .d(\a[25] ), .o1(new_n254));
  oaoi13aa1n06x5               g159(.a(new_n253), .b(new_n252), .c(\a[25] ), .d(\b[24] ), .o1(new_n255));
  norb02aa1n03x4               g160(.a(new_n254), .b(new_n255), .out0(\s[26] ));
  nor042aa1n06x5               g161(.a(new_n253), .b(new_n249), .o1(new_n257));
  nano22aa1n06x5               g162(.a(new_n223), .b(new_n239), .c(new_n257), .out0(new_n258));
  aoai13aa1n06x5               g163(.a(new_n258), .b(new_n238), .c(new_n122), .d(new_n174), .o1(new_n259));
  nanp02aa1n09x5               g164(.a(new_n247), .b(new_n257), .o1(new_n260));
  oai022aa1n02x5               g165(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n261));
  aob012aa1n02x5               g166(.a(new_n261), .b(\b[25] ), .c(\a[26] ), .out0(new_n262));
  nor042aa1n06x5               g167(.a(\b[26] ), .b(\a[27] ), .o1(new_n263));
  nanp02aa1n02x5               g168(.a(\b[26] ), .b(\a[27] ), .o1(new_n264));
  norb02aa1d27x5               g169(.a(new_n264), .b(new_n263), .out0(new_n265));
  inv000aa1d42x5               g170(.a(new_n265), .o1(new_n266));
  aoi013aa1n06x4               g171(.a(new_n266), .b(new_n259), .c(new_n260), .d(new_n262), .o1(new_n267));
  aoai13aa1n04x5               g172(.a(new_n239), .b(new_n226), .c(new_n210), .d(new_n222), .o1(new_n268));
  inv000aa1d42x5               g173(.a(new_n257), .o1(new_n269));
  aoai13aa1n06x5               g174(.a(new_n262), .b(new_n269), .c(new_n268), .d(new_n246), .o1(new_n270));
  aoi112aa1n02x5               g175(.a(new_n270), .b(new_n265), .c(new_n181), .d(new_n258), .o1(new_n271));
  norp02aa1n02x5               g176(.a(new_n267), .b(new_n271), .o1(\s[27] ));
  inv000aa1d42x5               g177(.a(new_n263), .o1(new_n273));
  xnrc02aa1n02x5               g178(.a(\b[27] ), .b(\a[28] ), .out0(new_n274));
  nano22aa1n03x5               g179(.a(new_n267), .b(new_n273), .c(new_n274), .out0(new_n275));
  inv000aa1n02x5               g180(.a(new_n258), .o1(new_n276));
  aoi012aa1n06x5               g181(.a(new_n276), .b(new_n175), .c(new_n180), .o1(new_n277));
  tech160nm_fioai012aa1n03p5x5 g182(.a(new_n265), .b(new_n270), .c(new_n277), .o1(new_n278));
  tech160nm_fiaoi012aa1n02p5x5 g183(.a(new_n274), .b(new_n278), .c(new_n273), .o1(new_n279));
  norp02aa1n03x5               g184(.a(new_n279), .b(new_n275), .o1(\s[28] ));
  nano22aa1n03x7               g185(.a(new_n274), .b(new_n273), .c(new_n264), .out0(new_n281));
  oaih12aa1n02x5               g186(.a(new_n281), .b(new_n270), .c(new_n277), .o1(new_n282));
  oao003aa1n02x5               g187(.a(\a[28] ), .b(\b[27] ), .c(new_n273), .carry(new_n283));
  xnrc02aa1n02x5               g188(.a(\b[28] ), .b(\a[29] ), .out0(new_n284));
  aoi012aa1n03x5               g189(.a(new_n284), .b(new_n282), .c(new_n283), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n281), .o1(new_n286));
  aoi013aa1n02x5               g191(.a(new_n286), .b(new_n259), .c(new_n260), .d(new_n262), .o1(new_n287));
  nano22aa1n03x5               g192(.a(new_n287), .b(new_n283), .c(new_n284), .out0(new_n288));
  norp02aa1n03x5               g193(.a(new_n285), .b(new_n288), .o1(\s[29] ));
  xorb03aa1n02x5               g194(.a(new_n104), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano23aa1n02x4               g195(.a(new_n284), .b(new_n274), .c(new_n264), .d(new_n273), .out0(new_n291));
  oaih12aa1n02x5               g196(.a(new_n291), .b(new_n270), .c(new_n277), .o1(new_n292));
  oao003aa1n02x5               g197(.a(\a[29] ), .b(\b[28] ), .c(new_n283), .carry(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[29] ), .b(\a[30] ), .out0(new_n294));
  aoi012aa1n02x5               g199(.a(new_n294), .b(new_n292), .c(new_n293), .o1(new_n295));
  inv000aa1n02x5               g200(.a(new_n291), .o1(new_n296));
  aoi013aa1n02x5               g201(.a(new_n296), .b(new_n259), .c(new_n260), .d(new_n262), .o1(new_n297));
  nano22aa1n03x5               g202(.a(new_n297), .b(new_n293), .c(new_n294), .out0(new_n298));
  norp02aa1n03x5               g203(.a(new_n295), .b(new_n298), .o1(\s[30] ));
  norb03aa1n02x5               g204(.a(new_n281), .b(new_n294), .c(new_n284), .out0(new_n300));
  inv000aa1n02x5               g205(.a(new_n300), .o1(new_n301));
  aoi013aa1n02x5               g206(.a(new_n301), .b(new_n259), .c(new_n260), .d(new_n262), .o1(new_n302));
  oao003aa1n02x5               g207(.a(\a[30] ), .b(\b[29] ), .c(new_n293), .carry(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[30] ), .b(\a[31] ), .out0(new_n304));
  nano22aa1n03x5               g209(.a(new_n302), .b(new_n303), .c(new_n304), .out0(new_n305));
  oaih12aa1n02x5               g210(.a(new_n300), .b(new_n270), .c(new_n277), .o1(new_n306));
  tech160nm_fiaoi012aa1n02p5x5 g211(.a(new_n304), .b(new_n306), .c(new_n303), .o1(new_n307));
  norp02aa1n03x5               g212(.a(new_n307), .b(new_n305), .o1(\s[31] ));
  inv000aa1d42x5               g213(.a(\a[3] ), .o1(new_n309));
  xorb03aa1n02x5               g214(.a(new_n138), .b(\b[2] ), .c(new_n309), .out0(\s[3] ));
  xorc02aa1n02x5               g215(.a(\a[4] ), .b(\b[3] ), .out0(new_n311));
  aoib12aa1n02x5               g216(.a(new_n311), .b(new_n309), .c(\b[2] ), .out0(new_n312));
  aoi022aa1n02x5               g217(.a(new_n139), .b(new_n311), .c(new_n106), .d(new_n312), .o1(\s[4] ));
  xorc02aa1n02x5               g218(.a(\a[5] ), .b(\b[4] ), .out0(new_n314));
  xobna2aa1n03x5               g219(.a(new_n314), .b(new_n139), .c(new_n112), .out0(\s[5] ));
  aoi013aa1n02x4               g220(.a(new_n116), .b(new_n139), .c(new_n112), .d(new_n314), .o1(new_n316));
  xnrb03aa1n02x5               g221(.a(new_n316), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g222(.a(\a[6] ), .b(\b[5] ), .c(new_n316), .o1(new_n318));
  xorb03aa1n02x5               g223(.a(new_n318), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  nanp02aa1n02x5               g224(.a(\b[6] ), .b(\a[7] ), .o1(new_n320));
  aoi012aa1n02x5               g225(.a(new_n111), .b(new_n318), .c(new_n320), .o1(new_n321));
  xnrc02aa1n02x5               g226(.a(new_n321), .b(new_n110), .out0(\s[8] ));
  xorb03aa1n02x5               g227(.a(new_n122), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


