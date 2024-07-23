// Benchmark "adder" written by ABC on Thu Jul 18 10:55:50 2024

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
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n149,
    new_n150, new_n151, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n162, new_n163, new_n164, new_n165,
    new_n166, new_n167, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n182, new_n183, new_n184, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n194, new_n195, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n213, new_n214,
    new_n215, new_n216, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n251, new_n252, new_n253,
    new_n254, new_n255, new_n256, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n313, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n320, new_n321, new_n322,
    new_n325, new_n328, new_n330, new_n331, new_n332;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n02x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nand22aa1n04x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  nand22aa1n04x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  aoi012aa1d24x5               g004(.a(new_n97), .b(new_n98), .c(new_n99), .o1(new_n100));
  nor042aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  tech160nm_finand02aa1n03p5x5 g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  norp02aa1n04x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nona23aa1n09x5               g009(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n105));
  tech160nm_fioai012aa1n05x5   g010(.a(new_n102), .b(new_n103), .c(new_n101), .o1(new_n106));
  oai012aa1n12x5               g011(.a(new_n106), .b(new_n105), .c(new_n100), .o1(new_n107));
  nand22aa1n12x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  nor042aa1n06x5               g013(.a(\b[6] ), .b(\a[7] ), .o1(new_n109));
  nand22aa1n12x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nanb03aa1d18x5               g015(.a(new_n109), .b(new_n110), .c(new_n108), .out0(new_n111));
  tech160nm_fixnrc02aa1n04x5   g016(.a(\b[7] ), .b(\a[8] ), .out0(new_n112));
  inv000aa1d42x5               g017(.a(\a[6] ), .o1(new_n113));
  inv000aa1d42x5               g018(.a(\b[5] ), .o1(new_n114));
  nand42aa1n03x5               g019(.a(new_n114), .b(new_n113), .o1(new_n115));
  nor042aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  nand42aa1n02x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  nanb03aa1n02x5               g022(.a(new_n116), .b(new_n115), .c(new_n117), .out0(new_n118));
  nor043aa1n06x5               g023(.a(new_n118), .b(new_n111), .c(new_n112), .o1(new_n119));
  oaih22aa1d12x5               g024(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n120));
  aoi013aa1n03x5               g025(.a(new_n109), .b(new_n120), .c(new_n110), .d(new_n108), .o1(new_n121));
  oaoi03aa1n09x5               g026(.a(\a[8] ), .b(\b[7] ), .c(new_n121), .o1(new_n122));
  aoi012aa1n12x5               g027(.a(new_n122), .b(new_n107), .c(new_n119), .o1(new_n123));
  oaoi03aa1n02x5               g028(.a(\a[9] ), .b(\b[8] ), .c(new_n123), .o1(new_n124));
  xorb03aa1n02x5               g029(.a(new_n124), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n04x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  nanp02aa1n02x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  nor022aa1n04x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nand22aa1n09x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nona23aa1n09x5               g034(.a(new_n129), .b(new_n127), .c(new_n126), .d(new_n128), .out0(new_n130));
  aoi012aa1n12x5               g035(.a(new_n128), .b(new_n126), .c(new_n129), .o1(new_n131));
  oai012aa1n02x5               g036(.a(new_n131), .b(new_n123), .c(new_n130), .o1(new_n132));
  xorb03aa1n02x5               g037(.a(new_n132), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1n04x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand22aa1n12x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  aoi012aa1n02x5               g040(.a(new_n134), .b(new_n132), .c(new_n135), .o1(new_n136));
  norp02aa1n09x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nand02aa1d04x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(new_n139));
  xnrc02aa1n02x5               g044(.a(new_n136), .b(new_n139), .out0(\s[12] ));
  nona23aa1n12x5               g045(.a(new_n138), .b(new_n135), .c(new_n134), .d(new_n137), .out0(new_n141));
  nor042aa1n03x5               g046(.a(new_n141), .b(new_n130), .o1(new_n142));
  aoai13aa1n06x5               g047(.a(new_n142), .b(new_n122), .c(new_n107), .d(new_n119), .o1(new_n143));
  nor042aa1n02x5               g048(.a(new_n137), .b(new_n134), .o1(new_n144));
  nanp02aa1n02x5               g049(.a(new_n131), .b(new_n144), .o1(new_n145));
  oaih12aa1n06x5               g050(.a(new_n138), .b(new_n137), .c(new_n135), .o1(new_n146));
  oaib12aa1n02x5               g051(.a(new_n143), .b(new_n146), .c(new_n145), .out0(new_n147));
  xorb03aa1n02x5               g052(.a(new_n147), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n12x5               g053(.a(\b[12] ), .b(\a[13] ), .o1(new_n149));
  nand42aa1n04x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  aoi012aa1n02x5               g055(.a(new_n149), .b(new_n147), .c(new_n150), .o1(new_n151));
  xnrb03aa1n02x5               g056(.a(new_n151), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n09x5               g057(.a(\b[13] ), .b(\a[14] ), .o1(new_n153));
  nand02aa1n03x5               g058(.a(\b[13] ), .b(\a[14] ), .o1(new_n154));
  nona23aa1n02x5               g059(.a(new_n154), .b(new_n150), .c(new_n149), .d(new_n153), .out0(new_n155));
  nano23aa1n06x5               g060(.a(new_n149), .b(new_n153), .c(new_n154), .d(new_n150), .out0(new_n156));
  nanb03aa1n03x5               g061(.a(new_n146), .b(new_n145), .c(new_n156), .out0(new_n157));
  oa0012aa1n12x5               g062(.a(new_n154), .b(new_n153), .c(new_n149), .o(new_n158));
  inv000aa1d42x5               g063(.a(new_n158), .o1(new_n159));
  oai112aa1n03x5               g064(.a(new_n157), .b(new_n159), .c(new_n143), .d(new_n155), .o1(new_n160));
  xorb03aa1n02x5               g065(.a(new_n160), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1n02x5               g066(.a(\b[14] ), .b(\a[15] ), .o1(new_n162));
  xnrc02aa1n12x5               g067(.a(\b[14] ), .b(\a[15] ), .out0(new_n163));
  inv000aa1d42x5               g068(.a(new_n163), .o1(new_n164));
  xnrc02aa1n06x5               g069(.a(\b[15] ), .b(\a[16] ), .out0(new_n165));
  aoai13aa1n02x5               g070(.a(new_n165), .b(new_n162), .c(new_n160), .d(new_n164), .o1(new_n166));
  aoi112aa1n02x5               g071(.a(new_n162), .b(new_n165), .c(new_n160), .d(new_n164), .o1(new_n167));
  nanb02aa1n03x5               g072(.a(new_n167), .b(new_n166), .out0(\s[16] ));
  nona22aa1n03x5               g073(.a(new_n156), .b(new_n163), .c(new_n165), .out0(new_n169));
  norb02aa1n03x5               g074(.a(new_n142), .b(new_n169), .out0(new_n170));
  aoai13aa1n06x5               g075(.a(new_n170), .b(new_n122), .c(new_n107), .d(new_n119), .o1(new_n171));
  aoi112aa1n06x5               g076(.a(new_n155), .b(new_n146), .c(new_n131), .d(new_n144), .o1(new_n172));
  norp02aa1n24x5               g077(.a(new_n165), .b(new_n163), .o1(new_n173));
  inv000aa1d42x5               g078(.a(\a[16] ), .o1(new_n174));
  inv000aa1d42x5               g079(.a(\b[15] ), .o1(new_n175));
  oao003aa1n09x5               g080(.a(new_n174), .b(new_n175), .c(new_n162), .carry(new_n176));
  oaoi13aa1n12x5               g081(.a(new_n176), .b(new_n173), .c(new_n172), .d(new_n158), .o1(new_n177));
  nor042aa1d18x5               g082(.a(\b[16] ), .b(\a[17] ), .o1(new_n178));
  nand42aa1n08x5               g083(.a(\b[16] ), .b(\a[17] ), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n179), .b(new_n178), .out0(new_n180));
  xnbna2aa1n03x5               g085(.a(new_n180), .b(new_n171), .c(new_n177), .out0(\s[17] ));
  nona23aa1d18x5               g086(.a(new_n156), .b(new_n173), .c(new_n141), .d(new_n130), .out0(new_n182));
  oai012aa1d24x5               g087(.a(new_n177), .b(new_n123), .c(new_n182), .o1(new_n183));
  tech160nm_fiaoi012aa1n05x5   g088(.a(new_n178), .b(new_n183), .c(new_n180), .o1(new_n184));
  xnrb03aa1n03x5               g089(.a(new_n184), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nor042aa1n09x5               g090(.a(\b[17] ), .b(\a[18] ), .o1(new_n186));
  nand02aa1d20x5               g091(.a(\b[17] ), .b(\a[18] ), .o1(new_n187));
  nano23aa1d15x5               g092(.a(new_n178), .b(new_n186), .c(new_n187), .d(new_n179), .out0(new_n188));
  inv000aa1d42x5               g093(.a(new_n188), .o1(new_n189));
  aoi012aa1n09x5               g094(.a(new_n186), .b(new_n178), .c(new_n187), .o1(new_n190));
  aoai13aa1n06x5               g095(.a(new_n190), .b(new_n189), .c(new_n171), .d(new_n177), .o1(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g097(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n06x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  xnrc02aa1n12x5               g099(.a(\b[18] ), .b(\a[19] ), .out0(new_n195));
  inv000aa1d42x5               g100(.a(new_n195), .o1(new_n196));
  nor042aa1n04x5               g101(.a(\b[19] ), .b(\a[20] ), .o1(new_n197));
  tech160nm_finand02aa1n03p5x5 g102(.a(\b[19] ), .b(\a[20] ), .o1(new_n198));
  nanb02aa1n09x5               g103(.a(new_n197), .b(new_n198), .out0(new_n199));
  aoai13aa1n03x5               g104(.a(new_n199), .b(new_n194), .c(new_n191), .d(new_n196), .o1(new_n200));
  nanp02aa1n02x5               g105(.a(new_n191), .b(new_n196), .o1(new_n201));
  nona22aa1n03x5               g106(.a(new_n201), .b(new_n199), .c(new_n194), .out0(new_n202));
  nanp02aa1n03x5               g107(.a(new_n202), .b(new_n200), .o1(\s[20] ));
  nona22aa1n12x5               g108(.a(new_n188), .b(new_n195), .c(new_n199), .out0(new_n204));
  tech160nm_finor002aa1n05x5   g109(.a(new_n197), .b(new_n194), .o1(new_n205));
  inv000aa1d42x5               g110(.a(\a[19] ), .o1(new_n206));
  inv000aa1d42x5               g111(.a(\b[18] ), .o1(new_n207));
  oai013aa1n03x5               g112(.a(new_n198), .b(new_n197), .c(new_n206), .d(new_n207), .o1(new_n208));
  aoi012aa1n12x5               g113(.a(new_n208), .b(new_n190), .c(new_n205), .o1(new_n209));
  inv000aa1d42x5               g114(.a(new_n209), .o1(new_n210));
  aoai13aa1n06x5               g115(.a(new_n210), .b(new_n204), .c(new_n171), .d(new_n177), .o1(new_n211));
  xorb03aa1n02x5               g116(.a(new_n211), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1d32x5               g117(.a(\b[20] ), .b(\a[21] ), .o1(new_n213));
  nand42aa1n16x5               g118(.a(\b[20] ), .b(\a[21] ), .o1(new_n214));
  norb02aa1n02x5               g119(.a(new_n214), .b(new_n213), .out0(new_n215));
  nor042aa1n06x5               g120(.a(\b[21] ), .b(\a[22] ), .o1(new_n216));
  nand42aa1n10x5               g121(.a(\b[21] ), .b(\a[22] ), .o1(new_n217));
  nanb02aa1n02x5               g122(.a(new_n216), .b(new_n217), .out0(new_n218));
  aoai13aa1n03x5               g123(.a(new_n218), .b(new_n213), .c(new_n211), .d(new_n215), .o1(new_n219));
  inv000aa1d42x5               g124(.a(new_n204), .o1(new_n220));
  aoai13aa1n04x5               g125(.a(new_n215), .b(new_n209), .c(new_n183), .d(new_n220), .o1(new_n221));
  nona22aa1n03x5               g126(.a(new_n221), .b(new_n218), .c(new_n213), .out0(new_n222));
  nanp02aa1n03x5               g127(.a(new_n219), .b(new_n222), .o1(\s[22] ));
  nano23aa1d15x5               g128(.a(new_n213), .b(new_n216), .c(new_n217), .d(new_n214), .out0(new_n224));
  nona23aa1d18x5               g129(.a(new_n224), .b(new_n188), .c(new_n195), .d(new_n199), .out0(new_n225));
  nanp02aa1n03x5               g130(.a(new_n190), .b(new_n205), .o1(new_n226));
  and002aa1n02x5               g131(.a(\b[18] ), .b(\a[19] ), .o(new_n227));
  oaoi03aa1n09x5               g132(.a(\a[20] ), .b(\b[19] ), .c(new_n227), .o1(new_n228));
  oa0012aa1n12x5               g133(.a(new_n217), .b(new_n216), .c(new_n213), .o(new_n229));
  aoi013aa1n02x4               g134(.a(new_n229), .b(new_n226), .c(new_n224), .d(new_n228), .o1(new_n230));
  aoai13aa1n06x5               g135(.a(new_n230), .b(new_n225), .c(new_n171), .d(new_n177), .o1(new_n231));
  xorb03aa1n02x5               g136(.a(new_n231), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g137(.a(\b[22] ), .b(\a[23] ), .o1(new_n233));
  xorc02aa1n12x5               g138(.a(\a[23] ), .b(\b[22] ), .out0(new_n234));
  tech160nm_fixnrc02aa1n05x5   g139(.a(\b[23] ), .b(\a[24] ), .out0(new_n235));
  aoai13aa1n03x5               g140(.a(new_n235), .b(new_n233), .c(new_n231), .d(new_n234), .o1(new_n236));
  nanp02aa1n02x5               g141(.a(new_n231), .b(new_n234), .o1(new_n237));
  nona22aa1n02x4               g142(.a(new_n237), .b(new_n235), .c(new_n233), .out0(new_n238));
  nanp02aa1n03x5               g143(.a(new_n238), .b(new_n236), .o1(\s[24] ));
  norb02aa1n12x5               g144(.a(new_n234), .b(new_n235), .out0(new_n240));
  nanb03aa1n03x5               g145(.a(new_n204), .b(new_n240), .c(new_n224), .out0(new_n241));
  nand23aa1n06x5               g146(.a(new_n226), .b(new_n224), .c(new_n228), .o1(new_n242));
  inv030aa1n02x5               g147(.a(new_n229), .o1(new_n243));
  inv000aa1n03x5               g148(.a(new_n240), .o1(new_n244));
  orn002aa1n02x5               g149(.a(\a[23] ), .b(\b[22] ), .o(new_n245));
  oao003aa1n02x5               g150(.a(\a[24] ), .b(\b[23] ), .c(new_n245), .carry(new_n246));
  aoai13aa1n12x5               g151(.a(new_n246), .b(new_n244), .c(new_n242), .d(new_n243), .o1(new_n247));
  inv000aa1d42x5               g152(.a(new_n247), .o1(new_n248));
  aoai13aa1n04x5               g153(.a(new_n248), .b(new_n241), .c(new_n171), .d(new_n177), .o1(new_n249));
  xorb03aa1n02x5               g154(.a(new_n249), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g155(.a(\b[24] ), .b(\a[25] ), .o1(new_n251));
  tech160nm_fixorc02aa1n03p5x5 g156(.a(\a[25] ), .b(\b[24] ), .out0(new_n252));
  xnrc02aa1n12x5               g157(.a(\b[25] ), .b(\a[26] ), .out0(new_n253));
  aoai13aa1n03x5               g158(.a(new_n253), .b(new_n251), .c(new_n249), .d(new_n252), .o1(new_n254));
  nand42aa1n02x5               g159(.a(new_n249), .b(new_n252), .o1(new_n255));
  nona22aa1n02x4               g160(.a(new_n255), .b(new_n253), .c(new_n251), .out0(new_n256));
  nanp02aa1n03x5               g161(.a(new_n256), .b(new_n254), .o1(\s[26] ));
  inv000aa1d42x5               g162(.a(new_n100), .o1(new_n258));
  nano23aa1n02x4               g163(.a(new_n101), .b(new_n103), .c(new_n104), .d(new_n102), .out0(new_n259));
  aobi12aa1n02x5               g164(.a(new_n106), .b(new_n259), .c(new_n258), .out0(new_n260));
  inv000aa1d42x5               g165(.a(\a[8] ), .o1(new_n261));
  inv000aa1d42x5               g166(.a(\b[7] ), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n108), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n109), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n110), .o1(new_n265));
  aoi012aa1n02x5               g170(.a(new_n116), .b(new_n113), .c(new_n114), .o1(new_n266));
  oai013aa1n02x4               g171(.a(new_n264), .b(new_n266), .c(new_n265), .d(new_n263), .o1(new_n267));
  oaoi03aa1n02x5               g172(.a(new_n261), .b(new_n262), .c(new_n267), .o1(new_n268));
  oaib12aa1n02x5               g173(.a(new_n268), .b(new_n260), .c(new_n119), .out0(new_n269));
  inv000aa1d42x5               g174(.a(new_n173), .o1(new_n270));
  inv000aa1d42x5               g175(.a(new_n176), .o1(new_n271));
  aoai13aa1n04x5               g176(.a(new_n271), .b(new_n270), .c(new_n157), .d(new_n159), .o1(new_n272));
  norb02aa1n09x5               g177(.a(new_n252), .b(new_n253), .out0(new_n273));
  nano22aa1d15x5               g178(.a(new_n225), .b(new_n240), .c(new_n273), .out0(new_n274));
  aoai13aa1n06x5               g179(.a(new_n274), .b(new_n272), .c(new_n269), .d(new_n170), .o1(new_n275));
  nanp02aa1n02x5               g180(.a(\b[25] ), .b(\a[26] ), .o1(new_n276));
  oai022aa1n02x5               g181(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n277));
  aoi022aa1n12x5               g182(.a(new_n247), .b(new_n273), .c(new_n276), .d(new_n277), .o1(new_n278));
  xorc02aa1n02x5               g183(.a(\a[27] ), .b(\b[26] ), .out0(new_n279));
  xnbna2aa1n03x5               g184(.a(new_n279), .b(new_n275), .c(new_n278), .out0(\s[27] ));
  inv000aa1d42x5               g185(.a(new_n274), .o1(new_n281));
  aoai13aa1n06x5               g186(.a(new_n278), .b(new_n281), .c(new_n171), .d(new_n177), .o1(new_n282));
  norp02aa1n02x5               g187(.a(\b[26] ), .b(\a[27] ), .o1(new_n283));
  norp02aa1n02x5               g188(.a(\b[27] ), .b(\a[28] ), .o1(new_n284));
  nand42aa1n03x5               g189(.a(\b[27] ), .b(\a[28] ), .o1(new_n285));
  norb02aa1n06x4               g190(.a(new_n285), .b(new_n284), .out0(new_n286));
  inv000aa1d42x5               g191(.a(new_n286), .o1(new_n287));
  aoai13aa1n02x7               g192(.a(new_n287), .b(new_n283), .c(new_n282), .d(new_n279), .o1(new_n288));
  aoai13aa1n03x5               g193(.a(new_n240), .b(new_n229), .c(new_n209), .d(new_n224), .o1(new_n289));
  inv000aa1d42x5               g194(.a(new_n273), .o1(new_n290));
  nanp02aa1n02x5               g195(.a(new_n277), .b(new_n276), .o1(new_n291));
  aoai13aa1n04x5               g196(.a(new_n291), .b(new_n290), .c(new_n289), .d(new_n246), .o1(new_n292));
  aoai13aa1n02x7               g197(.a(new_n279), .b(new_n292), .c(new_n183), .d(new_n274), .o1(new_n293));
  nona22aa1n03x5               g198(.a(new_n293), .b(new_n287), .c(new_n283), .out0(new_n294));
  nanp02aa1n03x5               g199(.a(new_n288), .b(new_n294), .o1(\s[28] ));
  norb02aa1n09x5               g200(.a(new_n279), .b(new_n287), .out0(new_n296));
  aoai13aa1n06x5               g201(.a(new_n296), .b(new_n292), .c(new_n183), .d(new_n274), .o1(new_n297));
  oai012aa1n02x5               g202(.a(new_n285), .b(new_n284), .c(new_n283), .o1(new_n298));
  xnrc02aa1n02x5               g203(.a(\b[28] ), .b(\a[29] ), .out0(new_n299));
  tech160nm_fiaoi012aa1n05x5   g204(.a(new_n299), .b(new_n297), .c(new_n298), .o1(new_n300));
  inv000aa1d42x5               g205(.a(new_n296), .o1(new_n301));
  tech160nm_fiaoi012aa1n04x5   g206(.a(new_n301), .b(new_n275), .c(new_n278), .o1(new_n302));
  nano22aa1n03x7               g207(.a(new_n302), .b(new_n298), .c(new_n299), .out0(new_n303));
  nor042aa1n03x5               g208(.a(new_n300), .b(new_n303), .o1(\s[29] ));
  xorb03aa1n02x5               g209(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n12x5               g210(.a(new_n299), .b(new_n279), .c(new_n286), .out0(new_n306));
  aoai13aa1n06x5               g211(.a(new_n306), .b(new_n292), .c(new_n183), .d(new_n274), .o1(new_n307));
  oao003aa1n02x5               g212(.a(\a[29] ), .b(\b[28] ), .c(new_n298), .carry(new_n308));
  xnrc02aa1n02x5               g213(.a(\b[29] ), .b(\a[30] ), .out0(new_n309));
  aoi012aa1n03x5               g214(.a(new_n309), .b(new_n307), .c(new_n308), .o1(new_n310));
  inv000aa1d42x5               g215(.a(new_n306), .o1(new_n311));
  tech160nm_fiaoi012aa1n02p5x5 g216(.a(new_n311), .b(new_n275), .c(new_n278), .o1(new_n312));
  nano22aa1n03x7               g217(.a(new_n312), .b(new_n308), .c(new_n309), .out0(new_n313));
  nor002aa1n02x5               g218(.a(new_n310), .b(new_n313), .o1(\s[30] ));
  nano23aa1n06x5               g219(.a(new_n309), .b(new_n299), .c(new_n279), .d(new_n286), .out0(new_n315));
  aoai13aa1n06x5               g220(.a(new_n315), .b(new_n292), .c(new_n183), .d(new_n274), .o1(new_n316));
  oao003aa1n02x5               g221(.a(\a[30] ), .b(\b[29] ), .c(new_n308), .carry(new_n317));
  xnrc02aa1n02x5               g222(.a(\b[30] ), .b(\a[31] ), .out0(new_n318));
  aoi012aa1n03x5               g223(.a(new_n318), .b(new_n316), .c(new_n317), .o1(new_n319));
  inv000aa1d42x5               g224(.a(new_n315), .o1(new_n320));
  tech160nm_fiaoi012aa1n02p5x5 g225(.a(new_n320), .b(new_n275), .c(new_n278), .o1(new_n321));
  nano22aa1n02x5               g226(.a(new_n321), .b(new_n317), .c(new_n318), .out0(new_n322));
  nor002aa1n02x5               g227(.a(new_n319), .b(new_n322), .o1(\s[31] ));
  xnrb03aa1n02x5               g228(.a(new_n100), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g229(.a(\a[3] ), .b(\b[2] ), .c(new_n100), .o1(new_n325));
  xorb03aa1n02x5               g230(.a(new_n325), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g231(.a(new_n107), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g232(.a(new_n116), .b(new_n107), .c(new_n117), .o1(new_n328));
  xnbna2aa1n03x5               g233(.a(new_n328), .b(new_n108), .c(new_n115), .out0(\s[6] ));
  norb03aa1n02x5               g234(.a(new_n117), .b(new_n260), .c(new_n116), .out0(new_n330));
  oabi12aa1n02x5               g235(.a(new_n111), .b(new_n330), .c(new_n120), .out0(new_n331));
  oai122aa1n02x7               g236(.a(new_n115), .b(new_n328), .c(new_n263), .d(new_n265), .e(new_n109), .o1(new_n332));
  and002aa1n02x5               g237(.a(new_n332), .b(new_n331), .o(\s[7] ));
  xobna2aa1n03x5               g238(.a(new_n112), .b(new_n331), .c(new_n264), .out0(\s[8] ));
  xnrb03aa1n02x5               g239(.a(new_n123), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


