// Benchmark "adder" written by ABC on Thu Jul 18 10:44:41 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n171, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n318, new_n321, new_n322, new_n324, new_n325, new_n326;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[10] ), .o1(new_n97));
  nor042aa1n03x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  oai022aa1n02x5               g003(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n99));
  xorc02aa1n12x5               g004(.a(\a[8] ), .b(\b[7] ), .out0(new_n100));
  nand22aa1n03x5               g005(.a(\b[5] ), .b(\a[6] ), .o1(new_n101));
  nor002aa1n20x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nand42aa1n02x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nano22aa1n03x7               g008(.a(new_n102), .b(new_n101), .c(new_n103), .out0(new_n104));
  inv000aa1d42x5               g009(.a(new_n102), .o1(new_n105));
  oaoi03aa1n02x5               g010(.a(\a[8] ), .b(\b[7] ), .c(new_n105), .o1(new_n106));
  aoi013aa1n03x5               g011(.a(new_n106), .b(new_n104), .c(new_n100), .d(new_n99), .o1(new_n107));
  norp02aa1n02x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[0] ), .b(\a[1] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[1] ), .b(\a[2] ), .o1(new_n110));
  tech160nm_fiaoi012aa1n04x5   g015(.a(new_n108), .b(new_n109), .c(new_n110), .o1(new_n111));
  nor002aa1n02x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[3] ), .b(\a[4] ), .o1(new_n113));
  nor022aa1n04x5               g018(.a(\b[2] ), .b(\a[3] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[2] ), .b(\a[3] ), .o1(new_n115));
  nona23aa1n03x5               g020(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n116));
  aoi012aa1n02x5               g021(.a(new_n112), .b(new_n114), .c(new_n113), .o1(new_n117));
  tech160nm_fioai012aa1n05x5   g022(.a(new_n117), .b(new_n116), .c(new_n111), .o1(new_n118));
  nanb03aa1n03x5               g023(.a(new_n102), .b(new_n103), .c(new_n101), .out0(new_n119));
  inv000aa1d42x5               g024(.a(\a[5] ), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\b[4] ), .o1(new_n121));
  nanp02aa1n02x5               g026(.a(new_n121), .b(new_n120), .o1(new_n122));
  nanp02aa1n02x5               g027(.a(\b[4] ), .b(\a[5] ), .o1(new_n123));
  oai112aa1n02x5               g028(.a(new_n122), .b(new_n123), .c(\b[5] ), .d(\a[6] ), .o1(new_n124));
  norb03aa1n03x5               g029(.a(new_n100), .b(new_n124), .c(new_n119), .out0(new_n125));
  nand02aa1d04x5               g030(.a(new_n118), .b(new_n125), .o1(new_n126));
  nand22aa1n06x5               g031(.a(new_n126), .b(new_n107), .o1(new_n127));
  xorc02aa1n12x5               g032(.a(\a[9] ), .b(\b[8] ), .out0(new_n128));
  aoi012aa1n02x5               g033(.a(new_n98), .b(new_n127), .c(new_n128), .o1(new_n129));
  xorb03aa1n02x5               g034(.a(new_n129), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  tech160nm_fixorc02aa1n02p5x5 g035(.a(\a[10] ), .b(\b[9] ), .out0(new_n131));
  aob012aa1n02x5               g036(.a(new_n98), .b(\b[9] ), .c(\a[10] ), .out0(new_n132));
  oaib12aa1n09x5               g037(.a(new_n132), .b(\b[9] ), .c(new_n97), .out0(new_n133));
  aoi013aa1n06x4               g038(.a(new_n133), .b(new_n127), .c(new_n128), .d(new_n131), .o1(new_n134));
  nor042aa1n06x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  inv000aa1d42x5               g040(.a(new_n135), .o1(new_n136));
  nand02aa1n03x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  xnbna2aa1n03x5               g042(.a(new_n134), .b(new_n137), .c(new_n136), .out0(\s[11] ));
  oaoi03aa1n03x5               g043(.a(\a[11] ), .b(\b[10] ), .c(new_n134), .o1(new_n139));
  xorb03aa1n02x5               g044(.a(new_n139), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nanp03aa1n02x5               g045(.a(new_n104), .b(new_n100), .c(new_n99), .o1(new_n141));
  nanb02aa1n02x5               g046(.a(new_n106), .b(new_n141), .out0(new_n142));
  norp02aa1n04x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nand02aa1n03x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nona23aa1n09x5               g049(.a(new_n144), .b(new_n137), .c(new_n135), .d(new_n143), .out0(new_n145));
  nano22aa1n03x7               g050(.a(new_n145), .b(new_n131), .c(new_n128), .out0(new_n146));
  aoai13aa1n02x5               g051(.a(new_n146), .b(new_n142), .c(new_n125), .d(new_n118), .o1(new_n147));
  inv000aa1d42x5               g052(.a(\b[9] ), .o1(new_n148));
  nanp02aa1n02x5               g053(.a(new_n148), .b(new_n97), .o1(new_n149));
  tech160nm_fiaoi012aa1n04x5   g054(.a(new_n143), .b(new_n135), .c(new_n144), .o1(new_n150));
  aoai13aa1n06x5               g055(.a(new_n150), .b(new_n145), .c(new_n149), .d(new_n132), .o1(new_n151));
  xnrc02aa1n12x5               g056(.a(\b[12] ), .b(\a[13] ), .out0(new_n152));
  aoib12aa1n02x5               g057(.a(new_n152), .b(new_n147), .c(new_n151), .out0(new_n153));
  inv000aa1d42x5               g058(.a(new_n152), .o1(new_n154));
  nona22aa1n02x4               g059(.a(new_n147), .b(new_n151), .c(new_n154), .out0(new_n155));
  norb02aa1n02x5               g060(.a(new_n155), .b(new_n153), .out0(\s[13] ));
  inv000aa1d42x5               g061(.a(\a[14] ), .o1(new_n157));
  norp02aa1n02x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  norp02aa1n03x5               g063(.a(new_n153), .b(new_n158), .o1(new_n159));
  xorb03aa1n02x5               g064(.a(new_n159), .b(\b[13] ), .c(new_n157), .out0(\s[14] ));
  xnrc02aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .out0(new_n161));
  nor042aa1n06x5               g066(.a(new_n161), .b(new_n152), .o1(new_n162));
  nano23aa1n06x5               g067(.a(new_n135), .b(new_n143), .c(new_n144), .d(new_n137), .out0(new_n163));
  inv030aa1n02x5               g068(.a(new_n150), .o1(new_n164));
  aoai13aa1n06x5               g069(.a(new_n162), .b(new_n164), .c(new_n163), .d(new_n133), .o1(new_n165));
  inv000aa1d42x5               g070(.a(\b[13] ), .o1(new_n166));
  oaoi03aa1n02x5               g071(.a(new_n157), .b(new_n166), .c(new_n158), .o1(new_n167));
  nanp02aa1n06x5               g072(.a(new_n165), .b(new_n167), .o1(new_n168));
  aoi013aa1n06x4               g073(.a(new_n168), .b(new_n127), .c(new_n146), .d(new_n162), .o1(new_n169));
  xnrb03aa1n02x5               g074(.a(new_n169), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  oaoi03aa1n03x5               g075(.a(\a[15] ), .b(\b[14] ), .c(new_n169), .o1(new_n171));
  xorb03aa1n02x5               g076(.a(new_n171), .b(\b[15] ), .c(\a[16] ), .out0(\s[16] ));
  inv000aa1n02x5               g077(.a(new_n167), .o1(new_n173));
  xnrc02aa1n12x5               g078(.a(\b[14] ), .b(\a[15] ), .out0(new_n174));
  tech160nm_fixnrc02aa1n05x5   g079(.a(\b[15] ), .b(\a[16] ), .out0(new_n175));
  nor042aa1n09x5               g080(.a(new_n175), .b(new_n174), .o1(new_n176));
  aoai13aa1n04x5               g081(.a(new_n176), .b(new_n173), .c(new_n151), .d(new_n162), .o1(new_n177));
  aoi112aa1n02x5               g082(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n178));
  oab012aa1n02x4               g083(.a(new_n178), .b(\a[16] ), .c(\b[15] ), .out0(new_n179));
  nand23aa1n03x5               g084(.a(new_n163), .b(new_n131), .c(new_n128), .o1(new_n180));
  nano22aa1n03x7               g085(.a(new_n180), .b(new_n162), .c(new_n176), .out0(new_n181));
  aoai13aa1n06x5               g086(.a(new_n181), .b(new_n142), .c(new_n125), .d(new_n118), .o1(new_n182));
  nand23aa1n06x5               g087(.a(new_n182), .b(new_n177), .c(new_n179), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g089(.a(\a[18] ), .o1(new_n185));
  inv000aa1d42x5               g090(.a(new_n176), .o1(new_n186));
  aoai13aa1n06x5               g091(.a(new_n179), .b(new_n186), .c(new_n165), .d(new_n167), .o1(new_n187));
  nanp03aa1n02x5               g092(.a(new_n146), .b(new_n162), .c(new_n176), .o1(new_n188));
  aoi012aa1n02x5               g093(.a(new_n188), .b(new_n126), .c(new_n107), .o1(new_n189));
  nor042aa1d18x5               g094(.a(\b[16] ), .b(\a[17] ), .o1(new_n190));
  nand42aa1n06x5               g095(.a(\b[16] ), .b(\a[17] ), .o1(new_n191));
  oaoi13aa1n02x5               g096(.a(new_n190), .b(new_n191), .c(new_n189), .d(new_n187), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[17] ), .c(new_n185), .out0(\s[18] ));
  aobi12aa1n06x5               g098(.a(new_n179), .b(new_n168), .c(new_n176), .out0(new_n194));
  nor002aa1n10x5               g099(.a(\b[17] ), .b(\a[18] ), .o1(new_n195));
  nand22aa1n12x5               g100(.a(\b[17] ), .b(\a[18] ), .o1(new_n196));
  nano23aa1d15x5               g101(.a(new_n190), .b(new_n195), .c(new_n196), .d(new_n191), .out0(new_n197));
  inv000aa1d42x5               g102(.a(new_n197), .o1(new_n198));
  aoi012aa1n09x5               g103(.a(new_n195), .b(new_n190), .c(new_n196), .o1(new_n199));
  aoai13aa1n04x5               g104(.a(new_n199), .b(new_n198), .c(new_n194), .d(new_n182), .o1(new_n200));
  xorb03aa1n02x5               g105(.a(new_n200), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g106(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d24x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  inv000aa1d42x5               g108(.a(new_n203), .o1(new_n204));
  inv000aa1n04x5               g109(.a(new_n199), .o1(new_n205));
  nand02aa1d08x5               g110(.a(\b[18] ), .b(\a[19] ), .o1(new_n206));
  norb02aa1n02x5               g111(.a(new_n206), .b(new_n203), .out0(new_n207));
  aoai13aa1n03x5               g112(.a(new_n207), .b(new_n205), .c(new_n183), .d(new_n197), .o1(new_n208));
  nor002aa1n10x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  nand22aa1n09x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  norb02aa1n02x5               g115(.a(new_n210), .b(new_n209), .out0(new_n211));
  inv000aa1d42x5               g116(.a(new_n211), .o1(new_n212));
  tech160nm_fiaoi012aa1n02p5x5 g117(.a(new_n212), .b(new_n208), .c(new_n204), .o1(new_n213));
  aoi112aa1n02x5               g118(.a(new_n203), .b(new_n211), .c(new_n200), .d(new_n207), .o1(new_n214));
  nor002aa1n02x5               g119(.a(new_n213), .b(new_n214), .o1(\s[20] ));
  nano23aa1d15x5               g120(.a(new_n203), .b(new_n209), .c(new_n210), .d(new_n206), .out0(new_n216));
  nand22aa1n12x5               g121(.a(new_n197), .b(new_n216), .o1(new_n217));
  nona23aa1d18x5               g122(.a(new_n210), .b(new_n206), .c(new_n203), .d(new_n209), .out0(new_n218));
  aoi012aa1n12x5               g123(.a(new_n209), .b(new_n203), .c(new_n210), .o1(new_n219));
  oai012aa1d24x5               g124(.a(new_n219), .b(new_n218), .c(new_n199), .o1(new_n220));
  inv000aa1d42x5               g125(.a(new_n220), .o1(new_n221));
  aoai13aa1n03x5               g126(.a(new_n221), .b(new_n217), .c(new_n194), .d(new_n182), .o1(new_n222));
  xorb03aa1n02x5               g127(.a(new_n222), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n06x5               g128(.a(\b[20] ), .b(\a[21] ), .o1(new_n224));
  inv000aa1n02x5               g129(.a(new_n224), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n217), .o1(new_n226));
  xorc02aa1n02x5               g131(.a(\a[21] ), .b(\b[20] ), .out0(new_n227));
  aoai13aa1n03x5               g132(.a(new_n227), .b(new_n220), .c(new_n183), .d(new_n226), .o1(new_n228));
  xorc02aa1n12x5               g133(.a(\a[22] ), .b(\b[21] ), .out0(new_n229));
  inv000aa1d42x5               g134(.a(new_n229), .o1(new_n230));
  tech160nm_fiaoi012aa1n02p5x5 g135(.a(new_n230), .b(new_n228), .c(new_n225), .o1(new_n231));
  aoi112aa1n03x4               g136(.a(new_n224), .b(new_n229), .c(new_n222), .d(new_n227), .o1(new_n232));
  nor002aa1n02x5               g137(.a(new_n231), .b(new_n232), .o1(\s[22] ));
  nano22aa1n02x5               g138(.a(new_n217), .b(new_n227), .c(new_n229), .out0(new_n234));
  inv000aa1n02x5               g139(.a(new_n234), .o1(new_n235));
  inv000aa1d42x5               g140(.a(\a[21] ), .o1(new_n236));
  inv020aa1n04x5               g141(.a(\a[22] ), .o1(new_n237));
  xroi22aa1d06x4               g142(.a(new_n236), .b(\b[20] ), .c(new_n237), .d(\b[21] ), .out0(new_n238));
  oao003aa1n09x5               g143(.a(\a[22] ), .b(\b[21] ), .c(new_n225), .carry(new_n239));
  inv000aa1d42x5               g144(.a(new_n239), .o1(new_n240));
  aoi012aa1d18x5               g145(.a(new_n240), .b(new_n220), .c(new_n238), .o1(new_n241));
  aoai13aa1n02x7               g146(.a(new_n241), .b(new_n235), .c(new_n194), .d(new_n182), .o1(new_n242));
  xorb03aa1n02x5               g147(.a(new_n242), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n06x5               g148(.a(\b[22] ), .b(\a[23] ), .o1(new_n244));
  inv000aa1d42x5               g149(.a(new_n244), .o1(new_n245));
  inv000aa1d42x5               g150(.a(new_n241), .o1(new_n246));
  tech160nm_fixorc02aa1n02p5x5 g151(.a(\a[23] ), .b(\b[22] ), .out0(new_n247));
  aoai13aa1n03x5               g152(.a(new_n247), .b(new_n246), .c(new_n183), .d(new_n234), .o1(new_n248));
  xorc02aa1n12x5               g153(.a(\a[24] ), .b(\b[23] ), .out0(new_n249));
  inv000aa1d42x5               g154(.a(new_n249), .o1(new_n250));
  tech160nm_fiaoi012aa1n02p5x5 g155(.a(new_n250), .b(new_n248), .c(new_n245), .o1(new_n251));
  aoi112aa1n03x4               g156(.a(new_n244), .b(new_n249), .c(new_n242), .d(new_n247), .o1(new_n252));
  nor002aa1n02x5               g157(.a(new_n251), .b(new_n252), .o1(\s[24] ));
  inv020aa1n03x5               g158(.a(new_n219), .o1(new_n254));
  aoai13aa1n06x5               g159(.a(new_n238), .b(new_n254), .c(new_n216), .d(new_n205), .o1(new_n255));
  and002aa1n03x5               g160(.a(new_n249), .b(new_n247), .o(new_n256));
  inv040aa1n03x5               g161(.a(new_n256), .o1(new_n257));
  oao003aa1n03x5               g162(.a(\a[24] ), .b(\b[23] ), .c(new_n245), .carry(new_n258));
  aoai13aa1n04x5               g163(.a(new_n258), .b(new_n257), .c(new_n255), .d(new_n239), .o1(new_n259));
  inv040aa1n03x5               g164(.a(new_n259), .o1(new_n260));
  nano32aa1n02x4               g165(.a(new_n257), .b(new_n238), .c(new_n197), .d(new_n216), .out0(new_n261));
  inv020aa1n02x5               g166(.a(new_n261), .o1(new_n262));
  aoai13aa1n02x7               g167(.a(new_n260), .b(new_n262), .c(new_n194), .d(new_n182), .o1(new_n263));
  xorb03aa1n02x5               g168(.a(new_n263), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g169(.a(\b[24] ), .b(\a[25] ), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n265), .o1(new_n266));
  xorc02aa1n12x5               g171(.a(\a[25] ), .b(\b[24] ), .out0(new_n267));
  aoai13aa1n03x5               g172(.a(new_n267), .b(new_n259), .c(new_n183), .d(new_n261), .o1(new_n268));
  xorc02aa1n12x5               g173(.a(\a[26] ), .b(\b[25] ), .out0(new_n269));
  inv000aa1d42x5               g174(.a(new_n269), .o1(new_n270));
  tech160nm_fiaoi012aa1n02p5x5 g175(.a(new_n270), .b(new_n268), .c(new_n266), .o1(new_n271));
  aoi112aa1n03x4               g176(.a(new_n265), .b(new_n269), .c(new_n263), .d(new_n267), .o1(new_n272));
  nor002aa1n02x5               g177(.a(new_n271), .b(new_n272), .o1(\s[26] ));
  and002aa1n24x5               g178(.a(new_n269), .b(new_n267), .o(new_n274));
  nano23aa1d15x5               g179(.a(new_n217), .b(new_n257), .c(new_n274), .d(new_n238), .out0(new_n275));
  aoai13aa1n06x5               g180(.a(new_n275), .b(new_n187), .c(new_n127), .d(new_n181), .o1(new_n276));
  oao003aa1n02x5               g181(.a(\a[26] ), .b(\b[25] ), .c(new_n266), .carry(new_n277));
  aobi12aa1n06x5               g182(.a(new_n277), .b(new_n259), .c(new_n274), .out0(new_n278));
  xorc02aa1n12x5               g183(.a(\a[27] ), .b(\b[26] ), .out0(new_n279));
  xnbna2aa1n03x5               g184(.a(new_n279), .b(new_n276), .c(new_n278), .out0(\s[27] ));
  norp02aa1n02x5               g185(.a(\b[26] ), .b(\a[27] ), .o1(new_n281));
  inv040aa1n03x5               g186(.a(new_n281), .o1(new_n282));
  aoai13aa1n03x5               g187(.a(new_n256), .b(new_n240), .c(new_n220), .d(new_n238), .o1(new_n283));
  inv000aa1d42x5               g188(.a(new_n274), .o1(new_n284));
  aoai13aa1n06x5               g189(.a(new_n277), .b(new_n284), .c(new_n283), .d(new_n258), .o1(new_n285));
  aoai13aa1n03x5               g190(.a(new_n279), .b(new_n285), .c(new_n183), .d(new_n275), .o1(new_n286));
  xnrc02aa1n02x5               g191(.a(\b[27] ), .b(\a[28] ), .out0(new_n287));
  tech160nm_fiaoi012aa1n02p5x5 g192(.a(new_n287), .b(new_n286), .c(new_n282), .o1(new_n288));
  aobi12aa1n02x7               g193(.a(new_n279), .b(new_n276), .c(new_n278), .out0(new_n289));
  nano22aa1n03x5               g194(.a(new_n289), .b(new_n282), .c(new_n287), .out0(new_n290));
  norp02aa1n03x5               g195(.a(new_n288), .b(new_n290), .o1(\s[28] ));
  xnrc02aa1n02x5               g196(.a(\b[28] ), .b(\a[29] ), .out0(new_n292));
  norb02aa1n02x5               g197(.a(new_n279), .b(new_n287), .out0(new_n293));
  aoai13aa1n03x5               g198(.a(new_n293), .b(new_n285), .c(new_n183), .d(new_n275), .o1(new_n294));
  oao003aa1n02x5               g199(.a(\a[28] ), .b(\b[27] ), .c(new_n282), .carry(new_n295));
  tech160nm_fiaoi012aa1n02p5x5 g200(.a(new_n292), .b(new_n294), .c(new_n295), .o1(new_n296));
  aobi12aa1n06x5               g201(.a(new_n293), .b(new_n276), .c(new_n278), .out0(new_n297));
  nano22aa1n03x5               g202(.a(new_n297), .b(new_n292), .c(new_n295), .out0(new_n298));
  norp02aa1n03x5               g203(.a(new_n296), .b(new_n298), .o1(\s[29] ));
  xorb03aa1n02x5               g204(.a(new_n109), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  xnrc02aa1n02x5               g205(.a(\b[29] ), .b(\a[30] ), .out0(new_n301));
  norb03aa1n02x5               g206(.a(new_n279), .b(new_n292), .c(new_n287), .out0(new_n302));
  aoai13aa1n03x5               g207(.a(new_n302), .b(new_n285), .c(new_n183), .d(new_n275), .o1(new_n303));
  oao003aa1n02x5               g208(.a(\a[29] ), .b(\b[28] ), .c(new_n295), .carry(new_n304));
  tech160nm_fiaoi012aa1n02p5x5 g209(.a(new_n301), .b(new_n303), .c(new_n304), .o1(new_n305));
  aobi12aa1n02x7               g210(.a(new_n302), .b(new_n276), .c(new_n278), .out0(new_n306));
  nano22aa1n03x5               g211(.a(new_n306), .b(new_n301), .c(new_n304), .out0(new_n307));
  norp02aa1n03x5               g212(.a(new_n305), .b(new_n307), .o1(\s[30] ));
  norb02aa1n02x5               g213(.a(new_n302), .b(new_n301), .out0(new_n309));
  aoai13aa1n03x5               g214(.a(new_n309), .b(new_n285), .c(new_n183), .d(new_n275), .o1(new_n310));
  oao003aa1n02x5               g215(.a(\a[30] ), .b(\b[29] ), .c(new_n304), .carry(new_n311));
  xnrc02aa1n02x5               g216(.a(\b[30] ), .b(\a[31] ), .out0(new_n312));
  tech160nm_fiaoi012aa1n02p5x5 g217(.a(new_n312), .b(new_n310), .c(new_n311), .o1(new_n313));
  aobi12aa1n02x7               g218(.a(new_n309), .b(new_n276), .c(new_n278), .out0(new_n314));
  nano22aa1n03x5               g219(.a(new_n314), .b(new_n311), .c(new_n312), .out0(new_n315));
  norp02aa1n03x5               g220(.a(new_n313), .b(new_n315), .o1(\s[31] ));
  xnrb03aa1n02x5               g221(.a(new_n111), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g222(.a(\a[3] ), .b(\b[2] ), .c(new_n111), .o1(new_n318));
  xorb03aa1n02x5               g223(.a(new_n318), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g224(.a(new_n118), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  orn002aa1n02x5               g225(.a(\a[6] ), .b(\b[5] ), .o(new_n321));
  oaoi03aa1n02x5               g226(.a(new_n120), .b(new_n121), .c(new_n118), .o1(new_n322));
  xnbna2aa1n03x5               g227(.a(new_n322), .b(new_n321), .c(new_n101), .out0(\s[6] ));
  nano22aa1n02x4               g228(.a(new_n322), .b(new_n321), .c(new_n101), .out0(new_n324));
  aob012aa1n02x5               g229(.a(new_n104), .b(new_n322), .c(new_n321), .out0(new_n325));
  oaib12aa1n02x5               g230(.a(new_n321), .b(new_n102), .c(new_n103), .out0(new_n326));
  oa0012aa1n02x5               g231(.a(new_n325), .b(new_n326), .c(new_n324), .o(\s[7] ));
  xnbna2aa1n03x5               g232(.a(new_n100), .b(new_n325), .c(new_n105), .out0(\s[8] ));
  xnbna2aa1n03x5               g233(.a(new_n128), .b(new_n126), .c(new_n107), .out0(\s[9] ));
endmodule


