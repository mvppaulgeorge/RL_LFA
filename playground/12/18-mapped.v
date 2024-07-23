// Benchmark "adder" written by ABC on Wed Jul 17 18:14:26 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n123, new_n124, new_n125,
    new_n126, new_n127, new_n128, new_n129, new_n131, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n233, new_n234, new_n235,
    new_n236, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n252, new_n253, new_n254, new_n255, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n307, new_n309, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n327, new_n329, new_n330, new_n332,
    new_n333, new_n335, new_n337, new_n339;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n06x5               g001(.a(\b[7] ), .b(\a[8] ), .o1(new_n97));
  nand22aa1n04x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  norp02aa1n12x5               g003(.a(\b[6] ), .b(\a[7] ), .o1(new_n99));
  nanp02aa1n04x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  nano23aa1n09x5               g005(.a(new_n97), .b(new_n99), .c(new_n100), .d(new_n98), .out0(new_n101));
  nona23aa1d18x5               g006(.a(new_n100), .b(new_n98), .c(new_n97), .d(new_n99), .out0(new_n102));
  oaih22aa1n06x5               g007(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n103));
  aob012aa1n12x5               g008(.a(new_n103), .b(\b[5] ), .c(\a[6] ), .out0(new_n104));
  oa0012aa1n03x5               g009(.a(new_n98), .b(new_n99), .c(new_n97), .o(new_n105));
  oabi12aa1n18x5               g010(.a(new_n105), .b(new_n102), .c(new_n104), .out0(new_n106));
  oai022aa1n09x5               g011(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n107));
  nor042aa1n03x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nand02aa1n04x5               g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  norb02aa1n06x5               g014(.a(new_n109), .b(new_n108), .out0(new_n110));
  norp02aa1n04x5               g015(.a(\b[1] ), .b(\a[2] ), .o1(new_n111));
  aoi022aa1d24x5               g016(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n112));
  oai012aa1n02x7               g017(.a(new_n110), .b(new_n112), .c(new_n111), .o1(new_n113));
  nanb02aa1n06x5               g018(.a(new_n107), .b(new_n113), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  and002aa1n02x7               g020(.a(\b[4] ), .b(\a[5] ), .o(new_n116));
  nanp02aa1n12x5               g021(.a(\b[3] ), .b(\a[4] ), .o1(new_n117));
  oai012aa1n06x5               g022(.a(new_n117), .b(\b[4] ), .c(\a[5] ), .o1(new_n118));
  norp03aa1n02x5               g023(.a(new_n115), .b(new_n116), .c(new_n118), .o1(new_n119));
  aoi013aa1n06x4               g024(.a(new_n106), .b(new_n114), .c(new_n119), .d(new_n101), .o1(new_n120));
  tech160nm_fioaoi03aa1n03p5x5 g025(.a(\a[9] ), .b(\b[8] ), .c(new_n120), .o1(new_n121));
  xorb03aa1n02x5               g026(.a(new_n121), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  inv000aa1d42x5               g027(.a(\a[11] ), .o1(new_n123));
  nand42aa1n16x5               g028(.a(\b[9] ), .b(\a[10] ), .o1(new_n124));
  nor002aa1d32x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nanb02aa1d36x5               g030(.a(new_n125), .b(new_n124), .out0(new_n126));
  inv000aa1d42x5               g031(.a(new_n126), .o1(new_n127));
  oai022aa1n02x5               g032(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n128));
  aoi022aa1n06x5               g033(.a(new_n121), .b(new_n127), .c(new_n124), .d(new_n128), .o1(new_n129));
  xorb03aa1n02x5               g034(.a(new_n129), .b(\b[10] ), .c(new_n123), .out0(\s[11] ));
  and002aa1n02x5               g035(.a(\b[10] ), .b(\a[11] ), .o(new_n131));
  xorc02aa1n12x5               g036(.a(\a[12] ), .b(\b[11] ), .out0(new_n132));
  nanb02aa1n02x5               g037(.a(\b[10] ), .b(new_n123), .out0(new_n133));
  nanp02aa1n02x5               g038(.a(new_n129), .b(new_n133), .o1(new_n134));
  nona22aa1n02x4               g039(.a(new_n134), .b(new_n132), .c(new_n131), .out0(new_n135));
  aoai13aa1n03x5               g040(.a(new_n132), .b(new_n131), .c(new_n129), .d(new_n133), .o1(new_n136));
  nanp02aa1n03x5               g041(.a(new_n135), .b(new_n136), .o1(\s[12] ));
  oaoi13aa1n12x5               g042(.a(new_n107), .b(new_n110), .c(new_n112), .d(new_n111), .o1(new_n138));
  xorc02aa1n02x5               g043(.a(\a[6] ), .b(\b[5] ), .out0(new_n139));
  nor042aa1n03x5               g044(.a(new_n118), .b(new_n116), .o1(new_n140));
  nanp02aa1n04x5               g045(.a(new_n140), .b(new_n139), .o1(new_n141));
  nor043aa1d12x5               g046(.a(new_n138), .b(new_n141), .c(new_n102), .o1(new_n142));
  tech160nm_fixnrc02aa1n04x5   g047(.a(\b[8] ), .b(\a[9] ), .out0(new_n143));
  xorc02aa1n12x5               g048(.a(\a[11] ), .b(\b[10] ), .out0(new_n144));
  nona23aa1d24x5               g049(.a(new_n132), .b(new_n144), .c(new_n143), .d(new_n126), .out0(new_n145));
  inv000aa1d42x5               g050(.a(new_n145), .o1(new_n146));
  oai012aa1n12x5               g051(.a(new_n146), .b(new_n142), .c(new_n106), .o1(new_n147));
  nanp02aa1n02x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  inv000aa1d42x5               g053(.a(\a[9] ), .o1(new_n149));
  inv000aa1d42x5               g054(.a(\b[8] ), .o1(new_n150));
  aoai13aa1n06x5               g055(.a(new_n124), .b(new_n125), .c(new_n149), .d(new_n150), .o1(new_n151));
  orn002aa1n02x5               g056(.a(\a[12] ), .b(\b[11] ), .o(new_n152));
  aoai13aa1n09x5               g057(.a(new_n152), .b(new_n131), .c(new_n151), .d(new_n133), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(new_n153), .b(new_n148), .o1(new_n154));
  nand22aa1n03x5               g059(.a(new_n147), .b(new_n154), .o1(new_n155));
  xorb03aa1n02x5               g060(.a(new_n155), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g061(.a(\a[13] ), .o1(new_n157));
  inv000aa1d42x5               g062(.a(\b[12] ), .o1(new_n158));
  oaoi03aa1n03x5               g063(.a(new_n157), .b(new_n158), .c(new_n155), .o1(new_n159));
  xnrb03aa1n03x5               g064(.a(new_n159), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1n02x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  nanp02aa1n02x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  norp02aa1n06x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nand02aa1n04x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nona23aa1n09x5               g069(.a(new_n164), .b(new_n162), .c(new_n161), .d(new_n163), .out0(new_n165));
  aoai13aa1n06x5               g070(.a(new_n164), .b(new_n163), .c(new_n157), .d(new_n158), .o1(new_n166));
  aoai13aa1n06x5               g071(.a(new_n166), .b(new_n165), .c(new_n147), .d(new_n154), .o1(new_n167));
  nor042aa1n03x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nanp02aa1n04x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  norb02aa1n02x5               g074(.a(new_n169), .b(new_n168), .out0(new_n170));
  oai022aa1n02x5               g075(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n171));
  aboi22aa1n03x5               g076(.a(new_n168), .b(new_n169), .c(new_n171), .d(new_n164), .out0(new_n172));
  aoai13aa1n02x5               g077(.a(new_n172), .b(new_n165), .c(new_n147), .d(new_n154), .o1(new_n173));
  aobi12aa1n02x5               g078(.a(new_n173), .b(new_n170), .c(new_n167), .out0(\s[15] ));
  nor042aa1n02x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  nanp02aa1n02x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  nanb02aa1n02x5               g081(.a(new_n175), .b(new_n176), .out0(new_n177));
  aoai13aa1n02x5               g082(.a(new_n177), .b(new_n168), .c(new_n167), .d(new_n169), .o1(new_n178));
  aoi112aa1n02x7               g083(.a(new_n168), .b(new_n177), .c(new_n167), .d(new_n169), .o1(new_n179));
  nanb02aa1n03x5               g084(.a(new_n179), .b(new_n178), .out0(\s[16] ));
  nona23aa1n09x5               g085(.a(new_n176), .b(new_n169), .c(new_n168), .d(new_n175), .out0(new_n181));
  nor042aa1n06x5               g086(.a(new_n181), .b(new_n165), .o1(new_n182));
  norb02aa1d21x5               g087(.a(new_n182), .b(new_n145), .out0(new_n183));
  oai012aa1d24x5               g088(.a(new_n183), .b(new_n142), .c(new_n106), .o1(new_n184));
  aoai13aa1n02x5               g089(.a(new_n169), .b(new_n168), .c(new_n171), .d(new_n164), .o1(new_n185));
  oaoi03aa1n02x5               g090(.a(\a[16] ), .b(\b[15] ), .c(new_n185), .o1(new_n186));
  aoi013aa1n09x5               g091(.a(new_n186), .b(new_n153), .c(new_n182), .d(new_n148), .o1(new_n187));
  xorc02aa1n06x5               g092(.a(\a[17] ), .b(\b[16] ), .out0(new_n188));
  xnbna2aa1n03x5               g093(.a(new_n188), .b(new_n184), .c(new_n187), .out0(\s[17] ));
  inv040aa1d32x5               g094(.a(\a[17] ), .o1(new_n190));
  inv000aa1d42x5               g095(.a(\b[16] ), .o1(new_n191));
  nanp02aa1n02x5               g096(.a(new_n191), .b(new_n190), .o1(new_n192));
  aoib12aa1n04x5               g097(.a(new_n105), .b(new_n101), .c(new_n104), .out0(new_n193));
  oai013aa1d12x5               g098(.a(new_n193), .b(new_n138), .c(new_n141), .d(new_n102), .o1(new_n194));
  nand23aa1n03x5               g099(.a(new_n153), .b(new_n182), .c(new_n148), .o1(new_n195));
  oaoi03aa1n02x5               g100(.a(\a[15] ), .b(\b[14] ), .c(new_n166), .o1(new_n196));
  tech160nm_fioai012aa1n04x5   g101(.a(new_n176), .b(new_n196), .c(new_n175), .o1(new_n197));
  nand22aa1n03x5               g102(.a(new_n195), .b(new_n197), .o1(new_n198));
  aoai13aa1n02x5               g103(.a(new_n188), .b(new_n198), .c(new_n194), .d(new_n183), .o1(new_n199));
  nor042aa1n03x5               g104(.a(\b[17] ), .b(\a[18] ), .o1(new_n200));
  nand42aa1n02x5               g105(.a(\b[17] ), .b(\a[18] ), .o1(new_n201));
  nanb02aa1n06x5               g106(.a(new_n200), .b(new_n201), .out0(new_n202));
  xobna2aa1n03x5               g107(.a(new_n202), .b(new_n199), .c(new_n192), .out0(\s[18] ));
  norb02aa1n02x5               g108(.a(new_n188), .b(new_n202), .out0(new_n204));
  aoai13aa1n06x5               g109(.a(new_n204), .b(new_n198), .c(new_n194), .d(new_n183), .o1(new_n205));
  aoai13aa1n06x5               g110(.a(new_n201), .b(new_n200), .c(new_n190), .d(new_n191), .o1(new_n206));
  nor042aa1n06x5               g111(.a(\b[18] ), .b(\a[19] ), .o1(new_n207));
  nand22aa1n04x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  norb02aa1n02x5               g113(.a(new_n208), .b(new_n207), .out0(new_n209));
  xnbna2aa1n03x5               g114(.a(new_n209), .b(new_n205), .c(new_n206), .out0(\s[19] ));
  xnrc02aa1n02x5               g115(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nand42aa1n03x5               g116(.a(new_n205), .b(new_n206), .o1(new_n212));
  nor002aa1n06x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  nand22aa1n09x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  nanb02aa1n02x5               g119(.a(new_n213), .b(new_n214), .out0(new_n215));
  aoai13aa1n02x5               g120(.a(new_n215), .b(new_n207), .c(new_n212), .d(new_n209), .o1(new_n216));
  nanp02aa1n06x5               g121(.a(new_n184), .b(new_n187), .o1(new_n217));
  oaoi03aa1n02x5               g122(.a(\a[18] ), .b(\b[17] ), .c(new_n192), .o1(new_n218));
  aoai13aa1n03x5               g123(.a(new_n209), .b(new_n218), .c(new_n217), .d(new_n204), .o1(new_n219));
  nona22aa1n03x5               g124(.a(new_n219), .b(new_n215), .c(new_n207), .out0(new_n220));
  nanp02aa1n03x5               g125(.a(new_n216), .b(new_n220), .o1(\s[20] ));
  nona23aa1d18x5               g126(.a(new_n214), .b(new_n208), .c(new_n207), .d(new_n213), .out0(new_n222));
  ao0012aa1n12x5               g127(.a(new_n213), .b(new_n207), .c(new_n214), .o(new_n223));
  oabi12aa1n18x5               g128(.a(new_n223), .b(new_n222), .c(new_n206), .out0(new_n224));
  inv000aa1d42x5               g129(.a(new_n224), .o1(new_n225));
  norb03aa1n09x5               g130(.a(new_n188), .b(new_n222), .c(new_n202), .out0(new_n226));
  inv000aa1d42x5               g131(.a(new_n226), .o1(new_n227));
  aoai13aa1n12x5               g132(.a(new_n225), .b(new_n227), .c(new_n184), .d(new_n187), .o1(new_n228));
  xnrc02aa1n12x5               g133(.a(\b[20] ), .b(\a[21] ), .out0(new_n229));
  inv000aa1d42x5               g134(.a(new_n229), .o1(new_n230));
  aoi012aa1n02x5               g135(.a(new_n230), .b(new_n217), .c(new_n226), .o1(new_n231));
  aoi022aa1n02x5               g136(.a(new_n231), .b(new_n225), .c(new_n228), .d(new_n230), .o1(\s[21] ));
  nor042aa1n03x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  tech160nm_fixnrc02aa1n04x5   g138(.a(\b[21] ), .b(\a[22] ), .out0(new_n234));
  aoai13aa1n03x5               g139(.a(new_n234), .b(new_n233), .c(new_n228), .d(new_n230), .o1(new_n235));
  aoi112aa1n03x5               g140(.a(new_n233), .b(new_n234), .c(new_n228), .d(new_n230), .o1(new_n236));
  nanb02aa1n03x5               g141(.a(new_n236), .b(new_n235), .out0(\s[22] ));
  nor042aa1n04x5               g142(.a(new_n234), .b(new_n229), .o1(new_n238));
  nona23aa1n09x5               g143(.a(new_n238), .b(new_n188), .c(new_n222), .d(new_n202), .out0(new_n239));
  nano23aa1n03x7               g144(.a(new_n207), .b(new_n213), .c(new_n214), .d(new_n208), .out0(new_n240));
  aoai13aa1n06x5               g145(.a(new_n238), .b(new_n223), .c(new_n240), .d(new_n218), .o1(new_n241));
  inv000aa1d42x5               g146(.a(\a[22] ), .o1(new_n242));
  inv000aa1d42x5               g147(.a(\b[21] ), .o1(new_n243));
  oaoi03aa1n12x5               g148(.a(new_n242), .b(new_n243), .c(new_n233), .o1(new_n244));
  nanp02aa1n02x5               g149(.a(new_n241), .b(new_n244), .o1(new_n245));
  inv000aa1n02x5               g150(.a(new_n245), .o1(new_n246));
  aoai13aa1n04x5               g151(.a(new_n246), .b(new_n239), .c(new_n184), .d(new_n187), .o1(new_n247));
  xorc02aa1n12x5               g152(.a(\a[23] ), .b(\b[22] ), .out0(new_n248));
  nanp02aa1n03x5               g153(.a(new_n247), .b(new_n248), .o1(new_n249));
  aoi113aa1n02x5               g154(.a(new_n245), .b(new_n248), .c(new_n217), .d(new_n226), .e(new_n238), .o1(new_n250));
  norb02aa1n02x7               g155(.a(new_n249), .b(new_n250), .out0(\s[23] ));
  norp02aa1n02x5               g156(.a(\b[22] ), .b(\a[23] ), .o1(new_n252));
  tech160nm_fixnrc02aa1n05x5   g157(.a(\b[23] ), .b(\a[24] ), .out0(new_n253));
  aoai13aa1n03x5               g158(.a(new_n253), .b(new_n252), .c(new_n247), .d(new_n248), .o1(new_n254));
  nona22aa1n03x5               g159(.a(new_n249), .b(new_n253), .c(new_n252), .out0(new_n255));
  nanp02aa1n02x5               g160(.a(new_n255), .b(new_n254), .o1(\s[24] ));
  norb02aa1n02x5               g161(.a(new_n248), .b(new_n253), .out0(new_n257));
  nand03aa1n02x5               g162(.a(new_n226), .b(new_n238), .c(new_n257), .o1(new_n258));
  inv000aa1n02x5               g163(.a(new_n257), .o1(new_n259));
  orn002aa1n02x5               g164(.a(\a[23] ), .b(\b[22] ), .o(new_n260));
  oao003aa1n02x5               g165(.a(\a[24] ), .b(\b[23] ), .c(new_n260), .carry(new_n261));
  aoai13aa1n06x5               g166(.a(new_n261), .b(new_n259), .c(new_n241), .d(new_n244), .o1(new_n262));
  inv040aa1n03x5               g167(.a(new_n262), .o1(new_n263));
  aoai13aa1n06x5               g168(.a(new_n263), .b(new_n258), .c(new_n184), .d(new_n187), .o1(new_n264));
  xorb03aa1n02x5               g169(.a(new_n264), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g170(.a(\b[24] ), .b(\a[25] ), .o1(new_n266));
  xorc02aa1n12x5               g171(.a(\a[25] ), .b(\b[24] ), .out0(new_n267));
  xnrc02aa1n12x5               g172(.a(\b[25] ), .b(\a[26] ), .out0(new_n268));
  aoai13aa1n03x5               g173(.a(new_n268), .b(new_n266), .c(new_n264), .d(new_n267), .o1(new_n269));
  nand42aa1n02x5               g174(.a(new_n264), .b(new_n267), .o1(new_n270));
  nona22aa1n03x5               g175(.a(new_n270), .b(new_n268), .c(new_n266), .out0(new_n271));
  nanp02aa1n03x5               g176(.a(new_n271), .b(new_n269), .o1(\s[26] ));
  norb02aa1n06x5               g177(.a(new_n267), .b(new_n268), .out0(new_n273));
  nano22aa1n06x5               g178(.a(new_n239), .b(new_n257), .c(new_n273), .out0(new_n274));
  aoai13aa1n06x5               g179(.a(new_n274), .b(new_n198), .c(new_n194), .d(new_n183), .o1(new_n275));
  nand42aa1n02x5               g180(.a(new_n262), .b(new_n273), .o1(new_n276));
  aoi112aa1n02x5               g181(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n277));
  oab012aa1n02x4               g182(.a(new_n277), .b(\a[26] ), .c(\b[25] ), .out0(new_n278));
  nanp03aa1n06x5               g183(.a(new_n275), .b(new_n276), .c(new_n278), .o1(new_n279));
  xorc02aa1n12x5               g184(.a(\a[27] ), .b(\b[26] ), .out0(new_n280));
  nano22aa1n02x4               g185(.a(new_n280), .b(new_n276), .c(new_n278), .out0(new_n281));
  aoi022aa1n02x5               g186(.a(new_n281), .b(new_n275), .c(new_n279), .d(new_n280), .o1(\s[27] ));
  nor002aa1n02x5               g187(.a(\b[26] ), .b(\a[27] ), .o1(new_n283));
  xnrc02aa1n12x5               g188(.a(\b[27] ), .b(\a[28] ), .out0(new_n284));
  aoai13aa1n03x5               g189(.a(new_n284), .b(new_n283), .c(new_n279), .d(new_n280), .o1(new_n285));
  aobi12aa1n06x5               g190(.a(new_n274), .b(new_n184), .c(new_n187), .out0(new_n286));
  inv000aa1n02x5               g191(.a(new_n244), .o1(new_n287));
  aoai13aa1n04x5               g192(.a(new_n257), .b(new_n287), .c(new_n224), .d(new_n238), .o1(new_n288));
  inv000aa1d42x5               g193(.a(new_n273), .o1(new_n289));
  aoai13aa1n06x5               g194(.a(new_n278), .b(new_n289), .c(new_n288), .d(new_n261), .o1(new_n290));
  oaih12aa1n02x5               g195(.a(new_n280), .b(new_n290), .c(new_n286), .o1(new_n291));
  nona22aa1n03x5               g196(.a(new_n291), .b(new_n284), .c(new_n283), .out0(new_n292));
  nanp02aa1n03x5               g197(.a(new_n285), .b(new_n292), .o1(\s[28] ));
  norb02aa1n03x5               g198(.a(new_n280), .b(new_n284), .out0(new_n294));
  oaih12aa1n02x5               g199(.a(new_n294), .b(new_n290), .c(new_n286), .o1(new_n295));
  xorc02aa1n12x5               g200(.a(\a[29] ), .b(\b[28] ), .out0(new_n296));
  inv000aa1d42x5               g201(.a(\a[28] ), .o1(new_n297));
  inv000aa1d42x5               g202(.a(\b[27] ), .o1(new_n298));
  aoi112aa1n02x5               g203(.a(\b[26] ), .b(\a[27] ), .c(\a[28] ), .d(\b[27] ), .o1(new_n299));
  aoi112aa1n02x5               g204(.a(new_n296), .b(new_n299), .c(new_n297), .d(new_n298), .o1(new_n300));
  aobi12aa1n06x5               g205(.a(new_n278), .b(new_n262), .c(new_n273), .out0(new_n301));
  inv000aa1d42x5               g206(.a(new_n294), .o1(new_n302));
  oao003aa1n06x5               g207(.a(new_n297), .b(new_n298), .c(new_n283), .carry(new_n303));
  inv000aa1d42x5               g208(.a(new_n303), .o1(new_n304));
  aoai13aa1n02x7               g209(.a(new_n304), .b(new_n302), .c(new_n301), .d(new_n275), .o1(new_n305));
  aoi022aa1n03x5               g210(.a(new_n305), .b(new_n296), .c(new_n295), .d(new_n300), .o1(\s[29] ));
  nanp02aa1n02x5               g211(.a(\b[0] ), .b(\a[1] ), .o1(new_n307));
  xorb03aa1n02x5               g212(.a(new_n307), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g213(.a(new_n284), .b(new_n280), .c(new_n296), .out0(new_n309));
  oaih12aa1n02x5               g214(.a(new_n309), .b(new_n290), .c(new_n286), .o1(new_n310));
  xorc02aa1n02x5               g215(.a(\a[30] ), .b(\b[29] ), .out0(new_n311));
  inv000aa1d42x5               g216(.a(\a[29] ), .o1(new_n312));
  inv000aa1d42x5               g217(.a(\b[28] ), .o1(new_n313));
  oabi12aa1n02x5               g218(.a(new_n311), .b(\a[29] ), .c(\b[28] ), .out0(new_n314));
  oaoi13aa1n02x5               g219(.a(new_n314), .b(new_n303), .c(new_n312), .d(new_n313), .o1(new_n315));
  inv000aa1n02x5               g220(.a(new_n309), .o1(new_n316));
  tech160nm_fioaoi03aa1n03p5x5 g221(.a(new_n312), .b(new_n313), .c(new_n303), .o1(new_n317));
  aoai13aa1n03x5               g222(.a(new_n317), .b(new_n316), .c(new_n301), .d(new_n275), .o1(new_n318));
  aoi022aa1n03x5               g223(.a(new_n318), .b(new_n311), .c(new_n310), .d(new_n315), .o1(\s[30] ));
  nanp03aa1n02x5               g224(.a(new_n294), .b(new_n296), .c(new_n311), .o1(new_n320));
  oabi12aa1n02x5               g225(.a(new_n320), .b(new_n290), .c(new_n286), .out0(new_n321));
  xorc02aa1n02x5               g226(.a(\a[31] ), .b(\b[30] ), .out0(new_n322));
  oao003aa1n02x5               g227(.a(\a[30] ), .b(\b[29] ), .c(new_n317), .carry(new_n323));
  norb02aa1n02x5               g228(.a(new_n323), .b(new_n322), .out0(new_n324));
  aoai13aa1n03x5               g229(.a(new_n323), .b(new_n320), .c(new_n301), .d(new_n275), .o1(new_n325));
  aoi022aa1n03x5               g230(.a(new_n325), .b(new_n322), .c(new_n321), .d(new_n324), .o1(\s[31] ));
  norp03aa1n02x5               g231(.a(new_n110), .b(new_n111), .c(new_n112), .o1(new_n327));
  norb02aa1n02x5               g232(.a(new_n113), .b(new_n327), .out0(\s[3] ));
  xorc02aa1n02x5               g233(.a(\a[4] ), .b(\b[3] ), .out0(new_n329));
  norp02aa1n02x5               g234(.a(new_n329), .b(new_n108), .o1(new_n330));
  aoi022aa1n02x5               g235(.a(new_n114), .b(new_n329), .c(new_n113), .d(new_n330), .o1(\s[4] ));
  xnrc02aa1n02x5               g236(.a(\b[4] ), .b(\a[5] ), .out0(new_n332));
  nanp02aa1n02x5               g237(.a(new_n114), .b(new_n117), .o1(new_n333));
  aoi022aa1n02x5               g238(.a(new_n333), .b(new_n332), .c(new_n114), .d(new_n140), .o1(\s[5] ));
  obai22aa1n02x7               g239(.a(new_n140), .b(new_n138), .c(\a[5] ), .d(\b[4] ), .out0(new_n335));
  xorb03aa1n02x5               g240(.a(new_n335), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oai012aa1n02x5               g241(.a(new_n104), .b(new_n138), .c(new_n141), .o1(new_n337));
  xorb03aa1n02x5               g242(.a(new_n337), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g243(.a(new_n99), .b(new_n337), .c(new_n100), .o1(new_n339));
  xnrb03aa1n02x5               g244(.a(new_n339), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g245(.a(new_n194), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


