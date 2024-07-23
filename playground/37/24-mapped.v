// Benchmark "adder" written by ABC on Thu Jul 18 07:07:26 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n169, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n193, new_n194, new_n195,
    new_n196, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n279, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n335, new_n338, new_n339, new_n340,
    new_n343;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor022aa1n16x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand42aa1n16x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n03x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  inv040aa1d28x5               g004(.a(\a[9] ), .o1(new_n100));
  inv030aa1d32x5               g005(.a(\b[8] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(new_n101), .b(new_n100), .o1(new_n102));
  tech160nm_fixnrc02aa1n02p5x5 g007(.a(\b[7] ), .b(\a[8] ), .out0(new_n103));
  nor022aa1n16x5               g008(.a(\b[6] ), .b(\a[7] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[6] ), .b(\a[7] ), .o1(new_n105));
  nanb02aa1n06x5               g010(.a(new_n104), .b(new_n105), .out0(new_n106));
  oai022aa1n02x7               g011(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n107));
  aob012aa1n02x5               g012(.a(new_n107), .b(\b[5] ), .c(\a[6] ), .out0(new_n108));
  inv000aa1d42x5               g013(.a(new_n104), .o1(new_n109));
  oao003aa1n02x5               g014(.a(\a[8] ), .b(\b[7] ), .c(new_n109), .carry(new_n110));
  oai013aa1n03x5               g015(.a(new_n110), .b(new_n108), .c(new_n103), .d(new_n106), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[1] ), .b(\a[2] ), .o1(new_n112));
  nand02aa1n02x5               g017(.a(\b[0] ), .b(\a[1] ), .o1(new_n113));
  norp02aa1n02x5               g018(.a(\b[1] ), .b(\a[2] ), .o1(new_n114));
  oai012aa1n02x7               g019(.a(new_n112), .b(new_n114), .c(new_n113), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[2] ), .b(\a[3] ), .o1(new_n116));
  norp02aa1n03x5               g021(.a(\b[2] ), .b(\a[3] ), .o1(new_n117));
  norp02aa1n02x5               g022(.a(\b[3] ), .b(\a[4] ), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(\b[3] ), .b(\a[4] ), .o1(new_n119));
  nona23aa1n02x5               g024(.a(new_n116), .b(new_n119), .c(new_n118), .d(new_n117), .out0(new_n120));
  oai012aa1n02x5               g025(.a(new_n119), .b(new_n118), .c(new_n117), .o1(new_n121));
  oaih12aa1n06x5               g026(.a(new_n121), .b(new_n120), .c(new_n115), .o1(new_n122));
  xnrc02aa1n12x5               g027(.a(\b[5] ), .b(\a[6] ), .out0(new_n123));
  norp02aa1n02x5               g028(.a(\b[4] ), .b(\a[5] ), .o1(new_n124));
  nanp02aa1n02x5               g029(.a(\b[4] ), .b(\a[5] ), .o1(new_n125));
  nona23aa1n02x4               g030(.a(new_n125), .b(new_n105), .c(new_n104), .d(new_n124), .out0(new_n126));
  nor043aa1n03x5               g031(.a(new_n126), .b(new_n123), .c(new_n103), .o1(new_n127));
  xorc02aa1n02x5               g032(.a(\a[9] ), .b(\b[8] ), .out0(new_n128));
  aoai13aa1n02x5               g033(.a(new_n128), .b(new_n111), .c(new_n122), .d(new_n127), .o1(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n99), .b(new_n129), .c(new_n102), .out0(\s[10] ));
  aobi12aa1n06x5               g035(.a(new_n99), .b(new_n129), .c(new_n102), .out0(new_n131));
  aoai13aa1n12x5               g036(.a(new_n98), .b(new_n97), .c(new_n100), .d(new_n101), .o1(new_n132));
  inv000aa1d42x5               g037(.a(new_n132), .o1(new_n133));
  xorc02aa1n02x5               g038(.a(\a[11] ), .b(\b[10] ), .out0(new_n134));
  inv000aa1d42x5               g039(.a(new_n134), .o1(new_n135));
  oab012aa1n02x4               g040(.a(new_n135), .b(new_n131), .c(new_n133), .out0(new_n136));
  nano22aa1n02x4               g041(.a(new_n131), .b(new_n132), .c(new_n135), .out0(new_n137));
  norp02aa1n02x5               g042(.a(new_n136), .b(new_n137), .o1(\s[11] ));
  nor042aa1n06x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  inv000aa1d42x5               g044(.a(new_n139), .o1(new_n140));
  oai012aa1n02x5               g045(.a(new_n134), .b(new_n131), .c(new_n133), .o1(new_n141));
  inv000aa1d42x5               g046(.a(\b[11] ), .o1(new_n142));
  nanb02aa1n02x5               g047(.a(\a[12] ), .b(new_n142), .out0(new_n143));
  nand42aa1n03x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nanp02aa1n02x5               g049(.a(new_n143), .b(new_n144), .o1(new_n145));
  aoi012aa1n02x5               g050(.a(new_n145), .b(new_n141), .c(new_n140), .o1(new_n146));
  nano22aa1n02x4               g051(.a(new_n136), .b(new_n140), .c(new_n145), .out0(new_n147));
  norp02aa1n02x5               g052(.a(new_n147), .b(new_n146), .o1(\s[12] ));
  nanp02aa1n02x5               g053(.a(\b[10] ), .b(\a[11] ), .o1(new_n149));
  nor002aa1n02x5               g054(.a(\b[11] ), .b(\a[12] ), .o1(new_n150));
  nanb03aa1n09x5               g055(.a(new_n150), .b(new_n144), .c(new_n149), .out0(new_n151));
  nano32aa1n02x4               g056(.a(new_n151), .b(new_n128), .c(new_n140), .d(new_n99), .out0(new_n152));
  aoai13aa1n06x5               g057(.a(new_n152), .b(new_n111), .c(new_n122), .d(new_n127), .o1(new_n153));
  tech160nm_fioai012aa1n03p5x5 g058(.a(new_n144), .b(new_n150), .c(new_n139), .o1(new_n154));
  oai013aa1d12x5               g059(.a(new_n154), .b(new_n151), .c(new_n132), .d(new_n139), .o1(new_n155));
  inv000aa1d42x5               g060(.a(new_n155), .o1(new_n156));
  nor042aa1n09x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  nanp02aa1n02x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  norb02aa1n03x5               g063(.a(new_n158), .b(new_n157), .out0(new_n159));
  xnbna2aa1n03x5               g064(.a(new_n159), .b(new_n153), .c(new_n156), .out0(\s[13] ));
  inv000aa1d42x5               g065(.a(new_n157), .o1(new_n161));
  norp03aa1n02x5               g066(.a(new_n108), .b(new_n103), .c(new_n106), .o1(new_n162));
  norb02aa1n02x5               g067(.a(new_n110), .b(new_n162), .out0(new_n163));
  nanp02aa1n02x5               g068(.a(new_n122), .b(new_n127), .o1(new_n164));
  nanp02aa1n02x5               g069(.a(new_n164), .b(new_n163), .o1(new_n165));
  aoai13aa1n02x5               g070(.a(new_n159), .b(new_n155), .c(new_n165), .d(new_n152), .o1(new_n166));
  nor002aa1n03x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  nanp02aa1n02x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  norb02aa1n06x4               g073(.a(new_n168), .b(new_n167), .out0(new_n169));
  xnbna2aa1n03x5               g074(.a(new_n169), .b(new_n166), .c(new_n161), .out0(\s[14] ));
  nona23aa1n02x4               g075(.a(new_n168), .b(new_n158), .c(new_n157), .d(new_n167), .out0(new_n171));
  oai012aa1n02x5               g076(.a(new_n168), .b(new_n167), .c(new_n157), .o1(new_n172));
  aoai13aa1n02x5               g077(.a(new_n172), .b(new_n171), .c(new_n153), .d(new_n156), .o1(new_n173));
  xorb03aa1n02x5               g078(.a(new_n173), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  inv000aa1d42x5               g079(.a(\a[15] ), .o1(new_n175));
  nanb02aa1d36x5               g080(.a(\b[14] ), .b(new_n175), .out0(new_n176));
  inv000aa1d42x5               g081(.a(new_n176), .o1(new_n177));
  nanp02aa1n02x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  xorc02aa1n02x5               g083(.a(\a[16] ), .b(\b[15] ), .out0(new_n179));
  aoai13aa1n03x5               g084(.a(new_n179), .b(new_n177), .c(new_n173), .d(new_n178), .o1(new_n180));
  aoi112aa1n02x5               g085(.a(new_n177), .b(new_n179), .c(new_n173), .d(new_n178), .o1(new_n181));
  norb02aa1n02x5               g086(.a(new_n180), .b(new_n181), .out0(\s[16] ));
  tech160nm_fiaoi012aa1n04x5   g087(.a(new_n111), .b(new_n122), .c(new_n127), .o1(new_n183));
  nand23aa1n03x5               g088(.a(new_n169), .b(new_n159), .c(new_n176), .o1(new_n184));
  nano22aa1n03x7               g089(.a(new_n184), .b(new_n178), .c(new_n179), .out0(new_n185));
  nanp02aa1n02x5               g090(.a(new_n152), .b(new_n185), .o1(new_n186));
  xnrc02aa1n02x5               g091(.a(\b[15] ), .b(\a[16] ), .out0(new_n187));
  nano23aa1n02x4               g092(.a(new_n187), .b(new_n172), .c(new_n176), .d(new_n178), .out0(new_n188));
  oaoi03aa1n02x5               g093(.a(\a[16] ), .b(\b[15] ), .c(new_n176), .o1(new_n189));
  aoi112aa1n09x5               g094(.a(new_n188), .b(new_n189), .c(new_n185), .d(new_n155), .o1(new_n190));
  oai012aa1n18x5               g095(.a(new_n190), .b(new_n183), .c(new_n186), .o1(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g097(.a(\a[18] ), .o1(new_n193));
  norp02aa1n02x5               g098(.a(\b[16] ), .b(\a[17] ), .o1(new_n194));
  xorc02aa1n12x5               g099(.a(\a[17] ), .b(\b[16] ), .out0(new_n195));
  tech160nm_fiaoi012aa1n05x5   g100(.a(new_n194), .b(new_n191), .c(new_n195), .o1(new_n196));
  xorb03aa1n02x5               g101(.a(new_n196), .b(\b[17] ), .c(new_n193), .out0(\s[18] ));
  nanp03aa1n02x5               g102(.a(new_n128), .b(new_n99), .c(new_n140), .o1(new_n198));
  inv000aa1n02x5               g103(.a(new_n151), .o1(new_n199));
  norb02aa1n02x5               g104(.a(new_n178), .b(new_n187), .out0(new_n200));
  nano23aa1n06x5               g105(.a(new_n198), .b(new_n184), .c(new_n200), .d(new_n199), .out0(new_n201));
  aoai13aa1n04x5               g106(.a(new_n201), .b(new_n111), .c(new_n127), .d(new_n122), .o1(new_n202));
  xorc02aa1n02x5               g107(.a(\a[18] ), .b(\b[17] ), .out0(new_n203));
  and002aa1n02x5               g108(.a(new_n203), .b(new_n195), .o(new_n204));
  inv000aa1n02x5               g109(.a(new_n204), .o1(new_n205));
  inv000aa1d42x5               g110(.a(\b[17] ), .o1(new_n206));
  oao003aa1n02x5               g111(.a(new_n193), .b(new_n206), .c(new_n194), .carry(new_n207));
  inv000aa1d42x5               g112(.a(new_n207), .o1(new_n208));
  aoai13aa1n02x7               g113(.a(new_n208), .b(new_n205), .c(new_n202), .d(new_n190), .o1(new_n209));
  xorb03aa1n02x5               g114(.a(new_n209), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g115(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g116(.a(\a[19] ), .o1(new_n212));
  inv000aa1d42x5               g117(.a(\b[18] ), .o1(new_n213));
  nand02aa1d06x5               g118(.a(new_n213), .b(new_n212), .o1(new_n214));
  xorc02aa1n02x5               g119(.a(\a[19] ), .b(\b[18] ), .out0(new_n215));
  aoai13aa1n03x5               g120(.a(new_n215), .b(new_n207), .c(new_n191), .d(new_n204), .o1(new_n216));
  nor042aa1n03x5               g121(.a(\b[19] ), .b(\a[20] ), .o1(new_n217));
  nanp02aa1n04x5               g122(.a(\b[19] ), .b(\a[20] ), .o1(new_n218));
  norb02aa1n02x5               g123(.a(new_n218), .b(new_n217), .out0(new_n219));
  aobi12aa1n02x7               g124(.a(new_n219), .b(new_n216), .c(new_n214), .out0(new_n220));
  inv000aa1d42x5               g125(.a(new_n214), .o1(new_n221));
  nanp02aa1n02x5               g126(.a(\b[18] ), .b(\a[19] ), .o1(new_n222));
  aoi112aa1n02x5               g127(.a(new_n221), .b(new_n219), .c(new_n209), .d(new_n222), .o1(new_n223));
  nor002aa1n02x5               g128(.a(new_n220), .b(new_n223), .o1(\s[20] ));
  nanb03aa1n12x5               g129(.a(new_n217), .b(new_n218), .c(new_n222), .out0(new_n225));
  nano32aa1n03x7               g130(.a(new_n225), .b(new_n203), .c(new_n195), .d(new_n214), .out0(new_n226));
  inv000aa1n02x5               g131(.a(new_n226), .o1(new_n227));
  nanp02aa1n02x5               g132(.a(\b[17] ), .b(\a[18] ), .o1(new_n228));
  oai022aa1n02x5               g133(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n229));
  nand23aa1n04x5               g134(.a(new_n229), .b(new_n214), .c(new_n228), .o1(new_n230));
  aoai13aa1n12x5               g135(.a(new_n218), .b(new_n217), .c(new_n212), .d(new_n213), .o1(new_n231));
  oai012aa1d24x5               g136(.a(new_n231), .b(new_n230), .c(new_n225), .o1(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  aoai13aa1n06x5               g138(.a(new_n233), .b(new_n227), .c(new_n202), .d(new_n190), .o1(new_n234));
  xorb03aa1n02x5               g139(.a(new_n234), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  xnrc02aa1n12x5               g140(.a(\b[20] ), .b(\a[21] ), .out0(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  aoai13aa1n06x5               g142(.a(new_n237), .b(new_n232), .c(new_n191), .d(new_n226), .o1(new_n238));
  xnrc02aa1n12x5               g143(.a(\b[21] ), .b(\a[22] ), .out0(new_n239));
  oaoi13aa1n06x5               g144(.a(new_n239), .b(new_n238), .c(\a[21] ), .d(\b[20] ), .o1(new_n240));
  norp02aa1n02x5               g145(.a(\b[20] ), .b(\a[21] ), .o1(new_n241));
  inv000aa1d42x5               g146(.a(new_n239), .o1(new_n242));
  aoi112aa1n03x4               g147(.a(new_n241), .b(new_n242), .c(new_n234), .d(new_n237), .o1(new_n243));
  norp02aa1n03x5               g148(.a(new_n240), .b(new_n243), .o1(\s[22] ));
  nano22aa1n02x4               g149(.a(new_n217), .b(new_n222), .c(new_n218), .out0(new_n245));
  nor042aa1n06x5               g150(.a(new_n239), .b(new_n236), .o1(new_n246));
  nano32aa1n02x4               g151(.a(new_n205), .b(new_n246), .c(new_n214), .d(new_n245), .out0(new_n247));
  inv000aa1n02x5               g152(.a(new_n247), .o1(new_n248));
  inv000aa1d42x5               g153(.a(\a[22] ), .o1(new_n249));
  inv000aa1d42x5               g154(.a(\b[21] ), .o1(new_n250));
  oao003aa1n02x5               g155(.a(new_n249), .b(new_n250), .c(new_n241), .carry(new_n251));
  aoi012aa1d18x5               g156(.a(new_n251), .b(new_n232), .c(new_n246), .o1(new_n252));
  aoai13aa1n02x7               g157(.a(new_n252), .b(new_n248), .c(new_n202), .d(new_n190), .o1(new_n253));
  xorb03aa1n02x5               g158(.a(new_n253), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g159(.a(\b[22] ), .b(\a[23] ), .o1(new_n255));
  inv000aa1n03x5               g160(.a(new_n255), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n252), .o1(new_n257));
  tech160nm_fixorc02aa1n05x5   g162(.a(\a[23] ), .b(\b[22] ), .out0(new_n258));
  aoai13aa1n03x5               g163(.a(new_n258), .b(new_n257), .c(new_n191), .d(new_n247), .o1(new_n259));
  tech160nm_fixorc02aa1n02p5x5 g164(.a(\a[24] ), .b(\b[23] ), .out0(new_n260));
  aobi12aa1n02x7               g165(.a(new_n260), .b(new_n259), .c(new_n256), .out0(new_n261));
  aoi112aa1n02x5               g166(.a(new_n255), .b(new_n260), .c(new_n253), .d(new_n258), .o1(new_n262));
  nor002aa1n02x5               g167(.a(new_n261), .b(new_n262), .o1(\s[24] ));
  oai012aa1n02x5               g168(.a(new_n228), .b(\b[18] ), .c(\a[19] ), .o1(new_n264));
  norb02aa1n02x5               g169(.a(new_n229), .b(new_n264), .out0(new_n265));
  inv000aa1d42x5               g170(.a(new_n231), .o1(new_n266));
  aoai13aa1n06x5               g171(.a(new_n246), .b(new_n266), .c(new_n265), .d(new_n245), .o1(new_n267));
  inv000aa1n02x5               g172(.a(new_n251), .o1(new_n268));
  and002aa1n06x5               g173(.a(new_n260), .b(new_n258), .o(new_n269));
  inv000aa1d42x5               g174(.a(new_n269), .o1(new_n270));
  oaoi03aa1n02x5               g175(.a(\a[24] ), .b(\b[23] ), .c(new_n256), .o1(new_n271));
  inv000aa1n02x5               g176(.a(new_n271), .o1(new_n272));
  aoai13aa1n06x5               g177(.a(new_n272), .b(new_n270), .c(new_n267), .d(new_n268), .o1(new_n273));
  inv040aa1n03x5               g178(.a(new_n273), .o1(new_n274));
  nano22aa1n02x4               g179(.a(new_n227), .b(new_n246), .c(new_n269), .out0(new_n275));
  inv000aa1n02x5               g180(.a(new_n275), .o1(new_n276));
  aoai13aa1n06x5               g181(.a(new_n274), .b(new_n276), .c(new_n202), .d(new_n190), .o1(new_n277));
  xorb03aa1n02x5               g182(.a(new_n277), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g183(.a(\b[24] ), .b(\a[25] ), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n279), .o1(new_n280));
  xorc02aa1n02x5               g185(.a(\a[25] ), .b(\b[24] ), .out0(new_n281));
  aoai13aa1n06x5               g186(.a(new_n281), .b(new_n273), .c(new_n191), .d(new_n275), .o1(new_n282));
  xorc02aa1n02x5               g187(.a(\a[26] ), .b(\b[25] ), .out0(new_n283));
  aobi12aa1n03x5               g188(.a(new_n283), .b(new_n282), .c(new_n280), .out0(new_n284));
  aoi112aa1n03x5               g189(.a(new_n279), .b(new_n283), .c(new_n277), .d(new_n281), .o1(new_n285));
  nor002aa1n02x5               g190(.a(new_n284), .b(new_n285), .o1(\s[26] ));
  nanp02aa1n02x5               g191(.a(new_n185), .b(new_n155), .o1(new_n287));
  nona22aa1n02x4               g192(.a(new_n287), .b(new_n189), .c(new_n188), .out0(new_n288));
  inv000aa1d42x5               g193(.a(\a[25] ), .o1(new_n289));
  inv000aa1d42x5               g194(.a(\a[26] ), .o1(new_n290));
  xroi22aa1d06x4               g195(.a(new_n289), .b(\b[24] ), .c(new_n290), .d(\b[25] ), .out0(new_n291));
  nano32aa1n03x7               g196(.a(new_n227), .b(new_n291), .c(new_n246), .d(new_n269), .out0(new_n292));
  aoai13aa1n06x5               g197(.a(new_n292), .b(new_n288), .c(new_n165), .d(new_n201), .o1(new_n293));
  oao003aa1n02x5               g198(.a(\a[26] ), .b(\b[25] ), .c(new_n280), .carry(new_n294));
  aobi12aa1n12x5               g199(.a(new_n294), .b(new_n273), .c(new_n291), .out0(new_n295));
  xorc02aa1n02x5               g200(.a(\a[27] ), .b(\b[26] ), .out0(new_n296));
  xnbna2aa1n03x5               g201(.a(new_n296), .b(new_n295), .c(new_n293), .out0(\s[27] ));
  norp02aa1n02x5               g202(.a(\b[26] ), .b(\a[27] ), .o1(new_n298));
  inv040aa1n03x5               g203(.a(new_n298), .o1(new_n299));
  aoai13aa1n09x5               g204(.a(new_n269), .b(new_n251), .c(new_n232), .d(new_n246), .o1(new_n300));
  inv000aa1d42x5               g205(.a(new_n291), .o1(new_n301));
  aoai13aa1n12x5               g206(.a(new_n294), .b(new_n301), .c(new_n300), .d(new_n272), .o1(new_n302));
  aoai13aa1n06x5               g207(.a(new_n296), .b(new_n302), .c(new_n191), .d(new_n292), .o1(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[27] ), .b(\a[28] ), .out0(new_n304));
  tech160nm_fiaoi012aa1n05x5   g209(.a(new_n304), .b(new_n303), .c(new_n299), .o1(new_n305));
  aobi12aa1n02x5               g210(.a(new_n296), .b(new_n295), .c(new_n293), .out0(new_n306));
  nano22aa1n02x4               g211(.a(new_n306), .b(new_n299), .c(new_n304), .out0(new_n307));
  nor002aa1n02x5               g212(.a(new_n305), .b(new_n307), .o1(\s[28] ));
  xnrc02aa1n02x5               g213(.a(\b[28] ), .b(\a[29] ), .out0(new_n309));
  norb02aa1n02x5               g214(.a(new_n296), .b(new_n304), .out0(new_n310));
  aoai13aa1n03x5               g215(.a(new_n310), .b(new_n302), .c(new_n191), .d(new_n292), .o1(new_n311));
  oao003aa1n02x5               g216(.a(\a[28] ), .b(\b[27] ), .c(new_n299), .carry(new_n312));
  aoi012aa1n03x5               g217(.a(new_n309), .b(new_n311), .c(new_n312), .o1(new_n313));
  aobi12aa1n02x5               g218(.a(new_n310), .b(new_n295), .c(new_n293), .out0(new_n314));
  nano22aa1n02x4               g219(.a(new_n314), .b(new_n309), .c(new_n312), .out0(new_n315));
  nor002aa1n02x5               g220(.a(new_n313), .b(new_n315), .o1(\s[29] ));
  xorb03aa1n02x5               g221(.a(new_n113), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  xnrc02aa1n02x5               g222(.a(\b[29] ), .b(\a[30] ), .out0(new_n318));
  norb03aa1n02x5               g223(.a(new_n296), .b(new_n309), .c(new_n304), .out0(new_n319));
  aoai13aa1n03x5               g224(.a(new_n319), .b(new_n302), .c(new_n191), .d(new_n292), .o1(new_n320));
  oao003aa1n03x5               g225(.a(\a[29] ), .b(\b[28] ), .c(new_n312), .carry(new_n321));
  aoi012aa1n03x5               g226(.a(new_n318), .b(new_n320), .c(new_n321), .o1(new_n322));
  aobi12aa1n02x7               g227(.a(new_n319), .b(new_n295), .c(new_n293), .out0(new_n323));
  nano22aa1n02x4               g228(.a(new_n323), .b(new_n318), .c(new_n321), .out0(new_n324));
  norp02aa1n03x5               g229(.a(new_n322), .b(new_n324), .o1(\s[30] ));
  xnrc02aa1n02x5               g230(.a(\b[30] ), .b(\a[31] ), .out0(new_n326));
  norb02aa1n02x5               g231(.a(new_n319), .b(new_n318), .out0(new_n327));
  aoai13aa1n03x5               g232(.a(new_n327), .b(new_n302), .c(new_n191), .d(new_n292), .o1(new_n328));
  oao003aa1n02x5               g233(.a(\a[30] ), .b(\b[29] ), .c(new_n321), .carry(new_n329));
  aoi012aa1n03x5               g234(.a(new_n326), .b(new_n328), .c(new_n329), .o1(new_n330));
  aobi12aa1n02x5               g235(.a(new_n327), .b(new_n295), .c(new_n293), .out0(new_n331));
  nano22aa1n02x4               g236(.a(new_n331), .b(new_n326), .c(new_n329), .out0(new_n332));
  nor002aa1n02x5               g237(.a(new_n330), .b(new_n332), .o1(\s[31] ));
  xnrb03aa1n02x5               g238(.a(new_n115), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g239(.a(\a[3] ), .b(\b[2] ), .c(new_n115), .o1(new_n335));
  xorb03aa1n02x5               g240(.a(new_n335), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g241(.a(new_n122), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  inv000aa1d42x5               g242(.a(new_n123), .o1(new_n338));
  aoai13aa1n02x5               g243(.a(new_n338), .b(new_n124), .c(new_n122), .d(new_n125), .o1(new_n339));
  aoi112aa1n02x5               g244(.a(new_n338), .b(new_n124), .c(new_n122), .d(new_n125), .o1(new_n340));
  norb02aa1n02x5               g245(.a(new_n339), .b(new_n340), .out0(\s[6] ));
  xobna2aa1n03x5               g246(.a(new_n106), .b(new_n339), .c(new_n108), .out0(\s[7] ));
  aoai13aa1n02x5               g247(.a(new_n109), .b(new_n106), .c(new_n339), .d(new_n108), .o1(new_n343));
  xorb03aa1n02x5               g248(.a(new_n343), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g249(.a(new_n183), .b(\b[8] ), .c(new_n100), .out0(\s[9] ));
endmodule


