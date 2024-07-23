// Benchmark "adder" written by ABC on Thu Jul 18 11:44:22 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n153, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n170,
    new_n171, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n181, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n202,
    new_n203, new_n204, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n277, new_n278, new_n279, new_n280, new_n281,
    new_n282, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n324, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n332, new_n333, new_n335, new_n338, new_n339, new_n341,
    new_n342, new_n344;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n04x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nand42aa1n03x5               g003(.a(\b[2] ), .b(\a[3] ), .o1(new_n99));
  nor002aa1n04x5               g004(.a(\b[2] ), .b(\a[3] ), .o1(new_n100));
  nand42aa1n06x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  nanb03aa1n06x5               g006(.a(new_n100), .b(new_n101), .c(new_n99), .out0(new_n102));
  nanp02aa1n02x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  nor042aa1n02x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  norb03aa1n03x5               g009(.a(new_n101), .b(new_n104), .c(new_n103), .out0(new_n105));
  xnrc02aa1n12x5               g010(.a(\b[3] ), .b(\a[4] ), .out0(new_n106));
  inv040aa1n02x5               g011(.a(new_n100), .o1(new_n107));
  oao003aa1n06x5               g012(.a(\a[4] ), .b(\b[3] ), .c(new_n107), .carry(new_n108));
  oai013aa1d12x5               g013(.a(new_n108), .b(new_n105), .c(new_n102), .d(new_n106), .o1(new_n109));
  xorc02aa1n02x5               g014(.a(\a[6] ), .b(\b[5] ), .out0(new_n110));
  xorc02aa1n02x5               g015(.a(\a[5] ), .b(\b[4] ), .out0(new_n111));
  xorc02aa1n12x5               g016(.a(\a[8] ), .b(\b[7] ), .out0(new_n112));
  nor042aa1n09x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  norb02aa1n06x4               g019(.a(new_n114), .b(new_n113), .out0(new_n115));
  nanp02aa1n02x5               g020(.a(new_n112), .b(new_n115), .o1(new_n116));
  nano22aa1n03x7               g021(.a(new_n116), .b(new_n110), .c(new_n111), .out0(new_n117));
  nanp02aa1n02x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  nano22aa1n02x4               g023(.a(new_n113), .b(new_n118), .c(new_n114), .out0(new_n119));
  oai022aa1n02x5               g024(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n120));
  nanp03aa1n02x5               g025(.a(new_n119), .b(new_n112), .c(new_n120), .o1(new_n121));
  inv000aa1d42x5               g026(.a(new_n113), .o1(new_n122));
  oaoi03aa1n02x5               g027(.a(\a[8] ), .b(\b[7] ), .c(new_n122), .o1(new_n123));
  nanb02aa1n12x5               g028(.a(new_n123), .b(new_n121), .out0(new_n124));
  xorc02aa1n12x5               g029(.a(\a[9] ), .b(\b[8] ), .out0(new_n125));
  aoai13aa1n02x5               g030(.a(new_n125), .b(new_n124), .c(new_n109), .d(new_n117), .o1(new_n126));
  nor002aa1n04x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nand42aa1d28x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  norb02aa1n06x5               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n129), .b(new_n126), .c(new_n98), .out0(\s[10] ));
  nona22aa1n02x4               g035(.a(new_n101), .b(new_n104), .c(new_n103), .out0(new_n131));
  nona22aa1n09x5               g036(.a(new_n131), .b(new_n102), .c(new_n106), .out0(new_n132));
  xnrc02aa1n02x5               g037(.a(\b[5] ), .b(\a[6] ), .out0(new_n133));
  xnrc02aa1n02x5               g038(.a(\b[4] ), .b(\a[5] ), .out0(new_n134));
  nona23aa1n02x4               g039(.a(new_n112), .b(new_n115), .c(new_n134), .d(new_n133), .out0(new_n135));
  aoi013aa1n02x4               g040(.a(new_n123), .b(new_n119), .c(new_n112), .d(new_n120), .o1(new_n136));
  aoai13aa1n04x5               g041(.a(new_n136), .b(new_n135), .c(new_n132), .d(new_n108), .o1(new_n137));
  aoai13aa1n02x5               g042(.a(new_n129), .b(new_n97), .c(new_n137), .d(new_n125), .o1(new_n138));
  inv000aa1d42x5               g043(.a(new_n128), .o1(new_n139));
  oab012aa1n04x5               g044(.a(new_n139), .b(new_n97), .c(new_n127), .out0(new_n140));
  inv000aa1d42x5               g045(.a(new_n140), .o1(new_n141));
  nor002aa1n06x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  nand42aa1n20x5               g047(.a(\b[10] ), .b(\a[11] ), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n143), .b(new_n142), .out0(new_n144));
  xnbna2aa1n03x5               g049(.a(new_n144), .b(new_n138), .c(new_n141), .out0(\s[11] ));
  nanp02aa1n02x5               g050(.a(new_n138), .b(new_n141), .o1(new_n146));
  nor002aa1n04x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  nand42aa1n16x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  nanb02aa1n02x5               g053(.a(new_n147), .b(new_n148), .out0(new_n149));
  aoai13aa1n02x5               g054(.a(new_n149), .b(new_n142), .c(new_n146), .d(new_n143), .o1(new_n150));
  nanp02aa1n02x5               g055(.a(new_n126), .b(new_n98), .o1(new_n151));
  aoai13aa1n02x5               g056(.a(new_n144), .b(new_n140), .c(new_n151), .d(new_n129), .o1(new_n152));
  nona22aa1n02x4               g057(.a(new_n152), .b(new_n149), .c(new_n142), .out0(new_n153));
  nanp02aa1n02x5               g058(.a(new_n150), .b(new_n153), .o1(\s[12] ));
  nano23aa1n06x5               g059(.a(new_n142), .b(new_n147), .c(new_n148), .d(new_n143), .out0(new_n155));
  nand03aa1n12x5               g060(.a(new_n155), .b(new_n125), .c(new_n129), .o1(new_n156));
  inv000aa1d42x5               g061(.a(new_n156), .o1(new_n157));
  aoai13aa1n06x5               g062(.a(new_n157), .b(new_n124), .c(new_n109), .d(new_n117), .o1(new_n158));
  nano22aa1n02x4               g063(.a(new_n147), .b(new_n143), .c(new_n148), .out0(new_n159));
  oai012aa1n02x5               g064(.a(new_n128), .b(\b[10] ), .c(\a[11] ), .o1(new_n160));
  oab012aa1n06x5               g065(.a(new_n160), .b(new_n97), .c(new_n127), .out0(new_n161));
  nanp02aa1n02x5               g066(.a(new_n161), .b(new_n159), .o1(new_n162));
  tech160nm_fiaoi012aa1n04x5   g067(.a(new_n147), .b(new_n142), .c(new_n148), .o1(new_n163));
  nanp02aa1n02x5               g068(.a(new_n162), .b(new_n163), .o1(new_n164));
  inv000aa1n02x5               g069(.a(new_n164), .o1(new_n165));
  nor002aa1n06x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  nand42aa1d28x5               g071(.a(\b[12] ), .b(\a[13] ), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n167), .b(new_n166), .out0(new_n168));
  xnbna2aa1n03x5               g073(.a(new_n168), .b(new_n158), .c(new_n165), .out0(\s[13] ));
  nanp02aa1n02x5               g074(.a(new_n158), .b(new_n165), .o1(new_n170));
  aoi012aa1n02x5               g075(.a(new_n166), .b(new_n170), .c(new_n167), .o1(new_n171));
  xnrb03aa1n02x5               g076(.a(new_n171), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n04x5               g077(.a(\b[13] ), .b(\a[14] ), .o1(new_n173));
  nand42aa1n20x5               g078(.a(\b[13] ), .b(\a[14] ), .o1(new_n174));
  nano23aa1d15x5               g079(.a(new_n166), .b(new_n173), .c(new_n174), .d(new_n167), .out0(new_n175));
  inv000aa1d42x5               g080(.a(new_n175), .o1(new_n176));
  oai012aa1n12x5               g081(.a(new_n174), .b(new_n173), .c(new_n166), .o1(new_n177));
  aoai13aa1n04x5               g082(.a(new_n177), .b(new_n176), .c(new_n158), .d(new_n165), .o1(new_n178));
  xorc02aa1n12x5               g083(.a(\a[15] ), .b(\b[14] ), .out0(new_n179));
  inv000aa1d42x5               g084(.a(new_n177), .o1(new_n180));
  aoi112aa1n02x5               g085(.a(new_n179), .b(new_n180), .c(new_n170), .d(new_n175), .o1(new_n181));
  aoi012aa1n02x5               g086(.a(new_n181), .b(new_n178), .c(new_n179), .o1(\s[15] ));
  norp02aa1n02x5               g087(.a(\b[14] ), .b(\a[15] ), .o1(new_n183));
  xorc02aa1n12x5               g088(.a(\a[16] ), .b(\b[15] ), .out0(new_n184));
  inv000aa1d42x5               g089(.a(new_n184), .o1(new_n185));
  aoai13aa1n02x5               g090(.a(new_n185), .b(new_n183), .c(new_n178), .d(new_n179), .o1(new_n186));
  aoi112aa1n02x5               g091(.a(new_n183), .b(new_n185), .c(new_n178), .d(new_n179), .o1(new_n187));
  nanb02aa1n03x5               g092(.a(new_n187), .b(new_n186), .out0(\s[16] ));
  nano32aa1d15x5               g093(.a(new_n156), .b(new_n184), .c(new_n175), .d(new_n179), .out0(new_n189));
  aoai13aa1n12x5               g094(.a(new_n189), .b(new_n124), .c(new_n109), .d(new_n117), .o1(new_n190));
  and002aa1n02x5               g095(.a(new_n184), .b(new_n179), .o(new_n191));
  inv000aa1n02x5               g096(.a(new_n163), .o1(new_n192));
  aoai13aa1n06x5               g097(.a(new_n175), .b(new_n192), .c(new_n161), .d(new_n159), .o1(new_n193));
  nand02aa1n02x5               g098(.a(new_n193), .b(new_n177), .o1(new_n194));
  orn002aa1n02x5               g099(.a(\a[15] ), .b(\b[14] ), .o(new_n195));
  oao003aa1n02x5               g100(.a(\a[16] ), .b(\b[15] ), .c(new_n195), .carry(new_n196));
  aobi12aa1n06x5               g101(.a(new_n196), .b(new_n194), .c(new_n191), .out0(new_n197));
  nor002aa1n16x5               g102(.a(\b[16] ), .b(\a[17] ), .o1(new_n198));
  nand42aa1n04x5               g103(.a(\b[16] ), .b(\a[17] ), .o1(new_n199));
  norb02aa1n02x5               g104(.a(new_n199), .b(new_n198), .out0(new_n200));
  xnbna2aa1n03x5               g105(.a(new_n200), .b(new_n197), .c(new_n190), .out0(\s[17] ));
  nand42aa1n02x5               g106(.a(new_n194), .b(new_n191), .o1(new_n202));
  nanp03aa1d12x5               g107(.a(new_n190), .b(new_n202), .c(new_n196), .o1(new_n203));
  tech160nm_fiaoi012aa1n05x5   g108(.a(new_n198), .b(new_n203), .c(new_n200), .o1(new_n204));
  xnrb03aa1n03x5               g109(.a(new_n204), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  inv000aa1n02x5               g110(.a(new_n191), .o1(new_n206));
  aoai13aa1n06x5               g111(.a(new_n196), .b(new_n206), .c(new_n193), .d(new_n177), .o1(new_n207));
  nor042aa1n06x5               g112(.a(\b[17] ), .b(\a[18] ), .o1(new_n208));
  nand42aa1n16x5               g113(.a(\b[17] ), .b(\a[18] ), .o1(new_n209));
  nano23aa1d15x5               g114(.a(new_n198), .b(new_n208), .c(new_n209), .d(new_n199), .out0(new_n210));
  aoai13aa1n06x5               g115(.a(new_n210), .b(new_n207), .c(new_n137), .d(new_n189), .o1(new_n211));
  oa0012aa1n02x5               g116(.a(new_n209), .b(new_n208), .c(new_n198), .o(new_n212));
  inv000aa1d42x5               g117(.a(new_n212), .o1(new_n213));
  nor002aa1n04x5               g118(.a(\b[18] ), .b(\a[19] ), .o1(new_n214));
  nanp02aa1n04x5               g119(.a(\b[18] ), .b(\a[19] ), .o1(new_n215));
  norb02aa1n09x5               g120(.a(new_n215), .b(new_n214), .out0(new_n216));
  xnbna2aa1n03x5               g121(.a(new_n216), .b(new_n211), .c(new_n213), .out0(\s[19] ));
  xnrc02aa1n02x5               g122(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanp02aa1n02x5               g123(.a(new_n211), .b(new_n213), .o1(new_n219));
  nor022aa1n08x5               g124(.a(\b[19] ), .b(\a[20] ), .o1(new_n220));
  nand22aa1n09x5               g125(.a(\b[19] ), .b(\a[20] ), .o1(new_n221));
  nanb02aa1n06x5               g126(.a(new_n220), .b(new_n221), .out0(new_n222));
  aoai13aa1n02x5               g127(.a(new_n222), .b(new_n214), .c(new_n219), .d(new_n215), .o1(new_n223));
  aoai13aa1n03x5               g128(.a(new_n216), .b(new_n212), .c(new_n203), .d(new_n210), .o1(new_n224));
  nona22aa1n03x5               g129(.a(new_n224), .b(new_n222), .c(new_n214), .out0(new_n225));
  nanp02aa1n03x5               g130(.a(new_n223), .b(new_n225), .o1(\s[20] ));
  nanb03aa1n03x5               g131(.a(new_n220), .b(new_n221), .c(new_n215), .out0(new_n227));
  oai122aa1n06x5               g132(.a(new_n209), .b(new_n208), .c(new_n198), .d(\b[18] ), .e(\a[19] ), .o1(new_n228));
  tech160nm_fiao0012aa1n05x5   g133(.a(new_n220), .b(new_n214), .c(new_n221), .o(new_n229));
  oabi12aa1n18x5               g134(.a(new_n229), .b(new_n228), .c(new_n227), .out0(new_n230));
  inv000aa1d42x5               g135(.a(new_n230), .o1(new_n231));
  nanb03aa1d18x5               g136(.a(new_n222), .b(new_n210), .c(new_n216), .out0(new_n232));
  aoai13aa1n04x5               g137(.a(new_n231), .b(new_n232), .c(new_n197), .d(new_n190), .o1(new_n233));
  nor042aa1n04x5               g138(.a(\b[20] ), .b(\a[21] ), .o1(new_n234));
  nanp02aa1n02x5               g139(.a(\b[20] ), .b(\a[21] ), .o1(new_n235));
  norb02aa1n02x5               g140(.a(new_n235), .b(new_n234), .out0(new_n236));
  inv000aa1d42x5               g141(.a(new_n232), .o1(new_n237));
  aoi012aa1n02x5               g142(.a(new_n236), .b(new_n203), .c(new_n237), .o1(new_n238));
  aoi022aa1n02x5               g143(.a(new_n238), .b(new_n231), .c(new_n233), .d(new_n236), .o1(\s[21] ));
  nor002aa1n03x5               g144(.a(\b[21] ), .b(\a[22] ), .o1(new_n240));
  nanp02aa1n02x5               g145(.a(\b[21] ), .b(\a[22] ), .o1(new_n241));
  nanb02aa1n02x5               g146(.a(new_n240), .b(new_n241), .out0(new_n242));
  aoai13aa1n02x5               g147(.a(new_n242), .b(new_n234), .c(new_n233), .d(new_n236), .o1(new_n243));
  aoai13aa1n03x5               g148(.a(new_n236), .b(new_n230), .c(new_n203), .d(new_n237), .o1(new_n244));
  nona22aa1n03x5               g149(.a(new_n244), .b(new_n242), .c(new_n234), .out0(new_n245));
  nanp02aa1n03x5               g150(.a(new_n243), .b(new_n245), .o1(\s[22] ));
  nano23aa1n06x5               g151(.a(new_n234), .b(new_n240), .c(new_n241), .d(new_n235), .out0(new_n247));
  nano32aa1n02x4               g152(.a(new_n222), .b(new_n247), .c(new_n210), .d(new_n216), .out0(new_n248));
  aoai13aa1n03x5               g153(.a(new_n248), .b(new_n207), .c(new_n137), .d(new_n189), .o1(new_n249));
  nano22aa1n03x5               g154(.a(new_n220), .b(new_n215), .c(new_n221), .out0(new_n250));
  tech160nm_fioai012aa1n05x5   g155(.a(new_n209), .b(\b[18] ), .c(\a[19] ), .o1(new_n251));
  oab012aa1n06x5               g156(.a(new_n251), .b(new_n198), .c(new_n208), .out0(new_n252));
  aoai13aa1n06x5               g157(.a(new_n247), .b(new_n229), .c(new_n252), .d(new_n250), .o1(new_n253));
  oa0012aa1n02x5               g158(.a(new_n241), .b(new_n240), .c(new_n234), .o(new_n254));
  inv000aa1n02x5               g159(.a(new_n254), .o1(new_n255));
  nanp02aa1n02x5               g160(.a(new_n253), .b(new_n255), .o1(new_n256));
  inv000aa1n02x5               g161(.a(new_n256), .o1(new_n257));
  xorc02aa1n12x5               g162(.a(\a[23] ), .b(\b[22] ), .out0(new_n258));
  xnbna2aa1n03x5               g163(.a(new_n258), .b(new_n249), .c(new_n257), .out0(\s[23] ));
  nand42aa1n03x5               g164(.a(new_n249), .b(new_n257), .o1(new_n260));
  norp02aa1n02x5               g165(.a(\b[22] ), .b(\a[23] ), .o1(new_n261));
  tech160nm_fixnrc02aa1n05x5   g166(.a(\b[23] ), .b(\a[24] ), .out0(new_n262));
  aoai13aa1n02x7               g167(.a(new_n262), .b(new_n261), .c(new_n260), .d(new_n258), .o1(new_n263));
  aoai13aa1n03x5               g168(.a(new_n258), .b(new_n256), .c(new_n203), .d(new_n248), .o1(new_n264));
  nona22aa1n03x5               g169(.a(new_n264), .b(new_n262), .c(new_n261), .out0(new_n265));
  nanp02aa1n03x5               g170(.a(new_n263), .b(new_n265), .o1(\s[24] ));
  norb02aa1n03x5               g171(.a(new_n258), .b(new_n262), .out0(new_n267));
  nano22aa1n03x7               g172(.a(new_n232), .b(new_n267), .c(new_n247), .out0(new_n268));
  inv000aa1n02x5               g173(.a(new_n268), .o1(new_n269));
  inv000aa1n02x5               g174(.a(new_n267), .o1(new_n270));
  oai022aa1n02x5               g175(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n271));
  aob012aa1n02x5               g176(.a(new_n271), .b(\b[23] ), .c(\a[24] ), .out0(new_n272));
  aoai13aa1n12x5               g177(.a(new_n272), .b(new_n270), .c(new_n253), .d(new_n255), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n273), .o1(new_n274));
  aoai13aa1n04x5               g179(.a(new_n274), .b(new_n269), .c(new_n197), .d(new_n190), .o1(new_n275));
  xorb03aa1n02x5               g180(.a(new_n275), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g181(.a(\b[24] ), .b(\a[25] ), .o1(new_n277));
  xorc02aa1n12x5               g182(.a(\a[25] ), .b(\b[24] ), .out0(new_n278));
  tech160nm_fixnrc02aa1n05x5   g183(.a(\b[25] ), .b(\a[26] ), .out0(new_n279));
  aoai13aa1n02x5               g184(.a(new_n279), .b(new_n277), .c(new_n275), .d(new_n278), .o1(new_n280));
  aoai13aa1n03x5               g185(.a(new_n278), .b(new_n273), .c(new_n203), .d(new_n268), .o1(new_n281));
  nona22aa1n03x5               g186(.a(new_n281), .b(new_n279), .c(new_n277), .out0(new_n282));
  nanp02aa1n03x5               g187(.a(new_n280), .b(new_n282), .o1(\s[26] ));
  norb02aa1n12x5               g188(.a(new_n278), .b(new_n279), .out0(new_n284));
  inv000aa1n02x5               g189(.a(new_n284), .o1(new_n285));
  nano23aa1n06x5               g190(.a(new_n285), .b(new_n232), .c(new_n267), .d(new_n247), .out0(new_n286));
  aoai13aa1n06x5               g191(.a(new_n286), .b(new_n207), .c(new_n137), .d(new_n189), .o1(new_n287));
  aoai13aa1n03x5               g192(.a(new_n267), .b(new_n254), .c(new_n230), .d(new_n247), .o1(new_n288));
  nanp02aa1n02x5               g193(.a(\b[25] ), .b(\a[26] ), .o1(new_n289));
  oai022aa1n02x5               g194(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n290));
  nanp02aa1n02x5               g195(.a(new_n290), .b(new_n289), .o1(new_n291));
  aoai13aa1n06x5               g196(.a(new_n291), .b(new_n285), .c(new_n288), .d(new_n272), .o1(new_n292));
  xorc02aa1n12x5               g197(.a(\a[27] ), .b(\b[26] ), .out0(new_n293));
  aoai13aa1n06x5               g198(.a(new_n293), .b(new_n292), .c(new_n203), .d(new_n286), .o1(new_n294));
  aoi122aa1n02x5               g199(.a(new_n293), .b(new_n289), .c(new_n290), .d(new_n273), .e(new_n284), .o1(new_n295));
  aobi12aa1n02x7               g200(.a(new_n294), .b(new_n295), .c(new_n287), .out0(\s[27] ));
  xnrc02aa1n02x5               g201(.a(\b[27] ), .b(\a[28] ), .out0(new_n297));
  aoi022aa1n09x5               g202(.a(new_n273), .b(new_n284), .c(new_n289), .d(new_n290), .o1(new_n298));
  norp02aa1n02x5               g203(.a(\b[26] ), .b(\a[27] ), .o1(new_n299));
  inv000aa1n03x5               g204(.a(new_n299), .o1(new_n300));
  inv000aa1d42x5               g205(.a(new_n293), .o1(new_n301));
  aoai13aa1n03x5               g206(.a(new_n300), .b(new_n301), .c(new_n287), .d(new_n298), .o1(new_n302));
  nanp02aa1n03x5               g207(.a(new_n302), .b(new_n297), .o1(new_n303));
  nona22aa1n02x5               g208(.a(new_n294), .b(new_n297), .c(new_n299), .out0(new_n304));
  nanp02aa1n03x5               g209(.a(new_n303), .b(new_n304), .o1(\s[28] ));
  norb02aa1n06x5               g210(.a(new_n293), .b(new_n297), .out0(new_n306));
  aoai13aa1n02x5               g211(.a(new_n306), .b(new_n292), .c(new_n203), .d(new_n286), .o1(new_n307));
  inv000aa1d42x5               g212(.a(new_n306), .o1(new_n308));
  oao003aa1n02x5               g213(.a(\a[28] ), .b(\b[27] ), .c(new_n300), .carry(new_n309));
  aoai13aa1n03x5               g214(.a(new_n309), .b(new_n308), .c(new_n287), .d(new_n298), .o1(new_n310));
  xorc02aa1n02x5               g215(.a(\a[29] ), .b(\b[28] ), .out0(new_n311));
  norb02aa1n02x5               g216(.a(new_n309), .b(new_n311), .out0(new_n312));
  aoi022aa1n03x5               g217(.a(new_n310), .b(new_n311), .c(new_n307), .d(new_n312), .o1(\s[29] ));
  xorb03aa1n02x5               g218(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x5               g219(.a(new_n297), .b(new_n293), .c(new_n311), .out0(new_n315));
  aoai13aa1n03x5               g220(.a(new_n315), .b(new_n292), .c(new_n203), .d(new_n286), .o1(new_n316));
  inv000aa1d42x5               g221(.a(new_n315), .o1(new_n317));
  oaoi03aa1n02x5               g222(.a(\a[29] ), .b(\b[28] ), .c(new_n309), .o1(new_n318));
  inv000aa1n03x5               g223(.a(new_n318), .o1(new_n319));
  aoai13aa1n02x7               g224(.a(new_n319), .b(new_n317), .c(new_n287), .d(new_n298), .o1(new_n320));
  xorc02aa1n02x5               g225(.a(\a[30] ), .b(\b[29] ), .out0(new_n321));
  norp02aa1n02x5               g226(.a(new_n318), .b(new_n321), .o1(new_n322));
  aoi022aa1n02x7               g227(.a(new_n320), .b(new_n321), .c(new_n316), .d(new_n322), .o1(\s[30] ));
  nano22aa1n06x5               g228(.a(new_n308), .b(new_n311), .c(new_n321), .out0(new_n324));
  aoai13aa1n02x5               g229(.a(new_n324), .b(new_n292), .c(new_n203), .d(new_n286), .o1(new_n325));
  xorc02aa1n02x5               g230(.a(\a[31] ), .b(\b[30] ), .out0(new_n326));
  oao003aa1n02x5               g231(.a(\a[30] ), .b(\b[29] ), .c(new_n319), .carry(new_n327));
  norb02aa1n02x5               g232(.a(new_n327), .b(new_n326), .out0(new_n328));
  inv000aa1d42x5               g233(.a(new_n324), .o1(new_n329));
  aoai13aa1n03x5               g234(.a(new_n327), .b(new_n329), .c(new_n287), .d(new_n298), .o1(new_n330));
  aoi022aa1n03x5               g235(.a(new_n330), .b(new_n326), .c(new_n325), .d(new_n328), .o1(\s[31] ));
  norp02aa1n02x5               g236(.a(new_n105), .b(new_n102), .o1(new_n332));
  aoi022aa1n02x5               g237(.a(new_n131), .b(new_n101), .c(new_n99), .d(new_n107), .o1(new_n333));
  norp02aa1n02x5               g238(.a(new_n333), .b(new_n332), .o1(\s[3] ));
  nano22aa1n02x4               g239(.a(new_n332), .b(new_n107), .c(new_n106), .out0(new_n335));
  oaoi13aa1n02x5               g240(.a(new_n335), .b(new_n109), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  xnbna2aa1n03x5               g241(.a(new_n111), .b(new_n132), .c(new_n108), .out0(\s[5] ));
  orn002aa1n02x5               g242(.a(\a[5] ), .b(\b[4] ), .o(new_n338));
  nanp02aa1n02x5               g243(.a(new_n109), .b(new_n111), .o1(new_n339));
  xnbna2aa1n03x5               g244(.a(new_n110), .b(new_n339), .c(new_n338), .out0(\s[6] ));
  nona22aa1n02x4               g245(.a(new_n109), .b(new_n133), .c(new_n134), .out0(new_n341));
  aob012aa1n02x5               g246(.a(new_n341), .b(new_n120), .c(new_n118), .out0(new_n342));
  xorb03aa1n02x5               g247(.a(new_n342), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  nanp02aa1n02x5               g248(.a(new_n342), .b(new_n115), .o1(new_n344));
  xnbna2aa1n03x5               g249(.a(new_n112), .b(new_n344), .c(new_n122), .out0(\s[8] ));
  xorb03aa1n02x5               g250(.a(new_n137), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule

