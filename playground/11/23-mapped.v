// Benchmark "adder" written by ABC on Wed Jul 17 17:46:45 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n125,
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n190, new_n191, new_n192, new_n193, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n255, new_n256, new_n257, new_n258, new_n259, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n313, new_n316, new_n317, new_n318, new_n320,
    new_n321, new_n322, new_n324;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  and002aa1n06x5               g002(.a(\b[3] ), .b(\a[4] ), .o(new_n98));
  xnrc02aa1n02x5               g003(.a(\b[2] ), .b(\a[3] ), .out0(new_n99));
  aoi022aa1n06x5               g004(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n100));
  oab012aa1n04x5               g005(.a(new_n100), .b(\a[2] ), .c(\b[1] ), .out0(new_n101));
  oa0022aa1n06x5               g006(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n102));
  oaoi13aa1n12x5               g007(.a(new_n98), .b(new_n102), .c(new_n101), .d(new_n99), .o1(new_n103));
  nor042aa1n02x5               g008(.a(\b[7] ), .b(\a[8] ), .o1(new_n104));
  nand22aa1n03x5               g009(.a(\b[7] ), .b(\a[8] ), .o1(new_n105));
  nor002aa1n03x5               g010(.a(\b[6] ), .b(\a[7] ), .o1(new_n106));
  nand02aa1d04x5               g011(.a(\b[6] ), .b(\a[7] ), .o1(new_n107));
  nona23aa1n03x5               g012(.a(new_n107), .b(new_n105), .c(new_n104), .d(new_n106), .out0(new_n108));
  nor022aa1n04x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nand42aa1d28x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  norb02aa1n03x5               g015(.a(new_n110), .b(new_n109), .out0(new_n111));
  xorc02aa1n02x5               g016(.a(\a[5] ), .b(\b[4] ), .out0(new_n112));
  nano22aa1n03x7               g017(.a(new_n108), .b(new_n112), .c(new_n111), .out0(new_n113));
  nanp02aa1n02x5               g018(.a(new_n103), .b(new_n113), .o1(new_n114));
  nano23aa1n06x5               g019(.a(new_n104), .b(new_n106), .c(new_n107), .d(new_n105), .out0(new_n115));
  aoi112aa1n02x5               g020(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n116));
  inv000aa1d42x5               g021(.a(\a[5] ), .o1(new_n117));
  inv000aa1d42x5               g022(.a(\b[4] ), .o1(new_n118));
  tech160nm_fiaoi012aa1n05x5   g023(.a(new_n109), .b(new_n117), .c(new_n118), .o1(new_n119));
  inv020aa1n02x5               g024(.a(new_n119), .o1(new_n120));
  aoi113aa1n03x7               g025(.a(new_n104), .b(new_n116), .c(new_n115), .d(new_n110), .e(new_n120), .o1(new_n121));
  xnrc02aa1n12x5               g026(.a(\b[8] ), .b(\a[9] ), .out0(new_n122));
  aoai13aa1n02x5               g027(.a(new_n97), .b(new_n122), .c(new_n114), .d(new_n121), .o1(new_n123));
  xorb03aa1n02x5               g028(.a(new_n123), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  inv000aa1d42x5               g029(.a(new_n98), .o1(new_n125));
  xorc02aa1n02x5               g030(.a(\a[3] ), .b(\b[2] ), .out0(new_n126));
  and002aa1n02x5               g031(.a(\b[0] ), .b(\a[1] ), .o(new_n127));
  oaoi03aa1n02x5               g032(.a(\a[2] ), .b(\b[1] ), .c(new_n127), .o1(new_n128));
  inv000aa1d42x5               g033(.a(new_n102), .o1(new_n129));
  aoai13aa1n06x5               g034(.a(new_n125), .b(new_n129), .c(new_n128), .d(new_n126), .o1(new_n130));
  nanp03aa1n02x5               g035(.a(new_n115), .b(new_n111), .c(new_n112), .o1(new_n131));
  oaih12aa1n02x5               g036(.a(new_n121), .b(new_n131), .c(new_n130), .o1(new_n132));
  tech160nm_fixnrc02aa1n02p5x5 g037(.a(\b[9] ), .b(\a[10] ), .out0(new_n133));
  nor002aa1n02x5               g038(.a(new_n122), .b(new_n133), .o1(new_n134));
  oai022aa1n04x7               g039(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n135));
  aob012aa1n02x5               g040(.a(new_n135), .b(\b[9] ), .c(\a[10] ), .out0(new_n136));
  aob012aa1n02x5               g041(.a(new_n136), .b(new_n132), .c(new_n134), .out0(new_n137));
  xorb03aa1n02x5               g042(.a(new_n137), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1n06x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  nand02aa1n06x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  nor042aa1n04x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand02aa1n06x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  aoi112aa1n03x5               g048(.a(new_n143), .b(new_n139), .c(new_n137), .d(new_n140), .o1(new_n144));
  aoai13aa1n04x5               g049(.a(new_n143), .b(new_n139), .c(new_n137), .d(new_n140), .o1(new_n145));
  norb02aa1n02x5               g050(.a(new_n145), .b(new_n144), .out0(\s[12] ));
  inv000aa1d42x5               g051(.a(new_n110), .o1(new_n147));
  nona22aa1n03x5               g052(.a(new_n115), .b(new_n147), .c(new_n119), .out0(new_n148));
  nona22aa1n03x5               g053(.a(new_n148), .b(new_n116), .c(new_n104), .out0(new_n149));
  nona23aa1n09x5               g054(.a(new_n142), .b(new_n140), .c(new_n139), .d(new_n141), .out0(new_n150));
  norp03aa1n02x5               g055(.a(new_n150), .b(new_n122), .c(new_n133), .o1(new_n151));
  aoai13aa1n02x5               g056(.a(new_n151), .b(new_n149), .c(new_n103), .d(new_n113), .o1(new_n152));
  aoi012aa1n12x5               g057(.a(new_n141), .b(new_n139), .c(new_n142), .o1(new_n153));
  oai012aa1n12x5               g058(.a(new_n153), .b(new_n150), .c(new_n136), .o1(new_n154));
  inv000aa1d42x5               g059(.a(new_n154), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(new_n152), .b(new_n155), .o1(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor022aa1n04x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nand42aa1n08x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  aoi012aa1n02x5               g064(.a(new_n158), .b(new_n156), .c(new_n159), .o1(new_n160));
  xnrb03aa1n02x5               g065(.a(new_n160), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n04x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nand42aa1n06x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nona23aa1n02x4               g068(.a(new_n163), .b(new_n159), .c(new_n158), .d(new_n162), .out0(new_n164));
  tech160nm_fioai012aa1n04x5   g069(.a(new_n163), .b(new_n162), .c(new_n158), .o1(new_n165));
  aoai13aa1n04x5               g070(.a(new_n165), .b(new_n164), .c(new_n152), .d(new_n155), .o1(new_n166));
  xorb03aa1n02x5               g071(.a(new_n166), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  xorc02aa1n12x5               g073(.a(\a[15] ), .b(\b[14] ), .out0(new_n169));
  xorc02aa1n12x5               g074(.a(\a[16] ), .b(\b[15] ), .out0(new_n170));
  aoi112aa1n02x5               g075(.a(new_n170), .b(new_n168), .c(new_n166), .d(new_n169), .o1(new_n171));
  aoai13aa1n02x5               g076(.a(new_n170), .b(new_n168), .c(new_n166), .d(new_n169), .o1(new_n172));
  norb02aa1n02x7               g077(.a(new_n172), .b(new_n171), .out0(\s[16] ));
  nano23aa1n06x5               g078(.a(new_n139), .b(new_n141), .c(new_n142), .d(new_n140), .out0(new_n174));
  nano23aa1n06x5               g079(.a(new_n158), .b(new_n162), .c(new_n163), .d(new_n159), .out0(new_n175));
  nanp03aa1n06x5               g080(.a(new_n175), .b(new_n169), .c(new_n170), .o1(new_n176));
  nano22aa1n03x7               g081(.a(new_n176), .b(new_n134), .c(new_n174), .out0(new_n177));
  aoai13aa1n12x5               g082(.a(new_n177), .b(new_n149), .c(new_n103), .d(new_n113), .o1(new_n178));
  nor002aa1n02x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  inv000aa1n02x5               g084(.a(new_n179), .o1(new_n180));
  nanb03aa1n06x5               g085(.a(new_n165), .b(new_n170), .c(new_n169), .out0(new_n181));
  aoi112aa1n03x5               g086(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n182));
  inv000aa1d42x5               g087(.a(new_n182), .o1(new_n183));
  oaoi03aa1n02x5               g088(.a(\a[10] ), .b(\b[9] ), .c(new_n97), .o1(new_n184));
  nanp02aa1n03x5               g089(.a(new_n174), .b(new_n184), .o1(new_n185));
  tech160nm_fiaoi012aa1n04x5   g090(.a(new_n176), .b(new_n185), .c(new_n153), .o1(new_n186));
  nano32aa1d12x5               g091(.a(new_n186), .b(new_n183), .c(new_n181), .d(new_n180), .out0(new_n187));
  nand02aa1d08x5               g092(.a(new_n178), .b(new_n187), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g094(.a(\a[18] ), .o1(new_n190));
  inv000aa1d42x5               g095(.a(\a[17] ), .o1(new_n191));
  inv000aa1d42x5               g096(.a(\b[16] ), .o1(new_n192));
  oaoi03aa1n02x5               g097(.a(new_n191), .b(new_n192), .c(new_n188), .o1(new_n193));
  xorb03aa1n02x5               g098(.a(new_n193), .b(\b[17] ), .c(new_n190), .out0(\s[18] ));
  xroi22aa1d04x5               g099(.a(new_n191), .b(\b[16] ), .c(new_n190), .d(\b[17] ), .out0(new_n195));
  oaih22aa1n06x5               g100(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n196));
  oaib12aa1n18x5               g101(.a(new_n196), .b(new_n190), .c(\b[17] ), .out0(new_n197));
  inv000aa1d42x5               g102(.a(new_n197), .o1(new_n198));
  nor042aa1n02x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nand02aa1d04x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  norb02aa1n02x5               g105(.a(new_n200), .b(new_n199), .out0(new_n201));
  aoai13aa1n06x5               g106(.a(new_n201), .b(new_n198), .c(new_n188), .d(new_n195), .o1(new_n202));
  aoi112aa1n02x5               g107(.a(new_n201), .b(new_n198), .c(new_n188), .d(new_n195), .o1(new_n203));
  norb02aa1n02x5               g108(.a(new_n202), .b(new_n203), .out0(\s[19] ));
  xnrc02aa1n02x5               g109(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1n03x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  nand02aa1n04x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  norb02aa1n02x5               g112(.a(new_n207), .b(new_n206), .out0(new_n208));
  nona22aa1n02x5               g113(.a(new_n202), .b(new_n208), .c(new_n199), .out0(new_n209));
  orn002aa1n02x5               g114(.a(\a[19] ), .b(\b[18] ), .o(new_n210));
  aobi12aa1n06x5               g115(.a(new_n208), .b(new_n202), .c(new_n210), .out0(new_n211));
  norb02aa1n03x4               g116(.a(new_n209), .b(new_n211), .out0(\s[20] ));
  nano23aa1n03x7               g117(.a(new_n199), .b(new_n206), .c(new_n207), .d(new_n200), .out0(new_n213));
  nanp02aa1n02x5               g118(.a(new_n195), .b(new_n213), .o1(new_n214));
  nona23aa1n09x5               g119(.a(new_n207), .b(new_n200), .c(new_n199), .d(new_n206), .out0(new_n215));
  aoi012aa1n06x5               g120(.a(new_n206), .b(new_n199), .c(new_n207), .o1(new_n216));
  oai012aa1n18x5               g121(.a(new_n216), .b(new_n215), .c(new_n197), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  aoai13aa1n06x5               g123(.a(new_n218), .b(new_n214), .c(new_n178), .d(new_n187), .o1(new_n219));
  xorb03aa1n02x5               g124(.a(new_n219), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g125(.a(\b[20] ), .b(\a[21] ), .o1(new_n221));
  xorc02aa1n02x5               g126(.a(\a[21] ), .b(\b[20] ), .out0(new_n222));
  xorc02aa1n02x5               g127(.a(\a[22] ), .b(\b[21] ), .out0(new_n223));
  aoi112aa1n03x5               g128(.a(new_n221), .b(new_n223), .c(new_n219), .d(new_n222), .o1(new_n224));
  aoai13aa1n03x5               g129(.a(new_n223), .b(new_n221), .c(new_n219), .d(new_n222), .o1(new_n225));
  norb02aa1n02x7               g130(.a(new_n225), .b(new_n224), .out0(\s[22] ));
  inv000aa1d42x5               g131(.a(\a[21] ), .o1(new_n227));
  inv000aa1d42x5               g132(.a(\a[22] ), .o1(new_n228));
  xroi22aa1d04x5               g133(.a(new_n227), .b(\b[20] ), .c(new_n228), .d(\b[21] ), .out0(new_n229));
  nanp03aa1n03x5               g134(.a(new_n229), .b(new_n195), .c(new_n213), .o1(new_n230));
  inv000aa1d42x5               g135(.a(\b[21] ), .o1(new_n231));
  oao003aa1n02x5               g136(.a(new_n228), .b(new_n231), .c(new_n221), .carry(new_n232));
  aoi012aa1n02x5               g137(.a(new_n232), .b(new_n217), .c(new_n229), .o1(new_n233));
  aoai13aa1n04x5               g138(.a(new_n233), .b(new_n230), .c(new_n178), .d(new_n187), .o1(new_n234));
  xorb03aa1n02x5               g139(.a(new_n234), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g140(.a(\b[22] ), .b(\a[23] ), .o1(new_n236));
  tech160nm_fixorc02aa1n02p5x5 g141(.a(\a[23] ), .b(\b[22] ), .out0(new_n237));
  xorc02aa1n02x5               g142(.a(\a[24] ), .b(\b[23] ), .out0(new_n238));
  aoi112aa1n02x7               g143(.a(new_n236), .b(new_n238), .c(new_n234), .d(new_n237), .o1(new_n239));
  aoai13aa1n03x5               g144(.a(new_n238), .b(new_n236), .c(new_n234), .d(new_n237), .o1(new_n240));
  norb02aa1n02x7               g145(.a(new_n240), .b(new_n239), .out0(\s[24] ));
  and002aa1n02x5               g146(.a(new_n238), .b(new_n237), .o(new_n242));
  inv000aa1n02x5               g147(.a(new_n242), .o1(new_n243));
  nano32aa1n02x4               g148(.a(new_n243), .b(new_n229), .c(new_n195), .d(new_n213), .out0(new_n244));
  inv000aa1n02x5               g149(.a(new_n216), .o1(new_n245));
  aoai13aa1n03x5               g150(.a(new_n229), .b(new_n245), .c(new_n213), .d(new_n198), .o1(new_n246));
  inv000aa1d42x5               g151(.a(new_n232), .o1(new_n247));
  aoi112aa1n02x5               g152(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n248));
  oab012aa1n02x4               g153(.a(new_n248), .b(\a[24] ), .c(\b[23] ), .out0(new_n249));
  aoai13aa1n06x5               g154(.a(new_n249), .b(new_n243), .c(new_n246), .d(new_n247), .o1(new_n250));
  tech160nm_fixorc02aa1n05x5   g155(.a(\a[25] ), .b(\b[24] ), .out0(new_n251));
  aoai13aa1n06x5               g156(.a(new_n251), .b(new_n250), .c(new_n188), .d(new_n244), .o1(new_n252));
  aoi112aa1n02x5               g157(.a(new_n251), .b(new_n250), .c(new_n188), .d(new_n244), .o1(new_n253));
  norb02aa1n02x5               g158(.a(new_n252), .b(new_n253), .out0(\s[25] ));
  nor042aa1n03x5               g159(.a(\b[24] ), .b(\a[25] ), .o1(new_n255));
  tech160nm_fixorc02aa1n02p5x5 g160(.a(\a[26] ), .b(\b[25] ), .out0(new_n256));
  nona22aa1n02x5               g161(.a(new_n252), .b(new_n256), .c(new_n255), .out0(new_n257));
  inv000aa1d42x5               g162(.a(new_n255), .o1(new_n258));
  aobi12aa1n06x5               g163(.a(new_n256), .b(new_n252), .c(new_n258), .out0(new_n259));
  norb02aa1n03x4               g164(.a(new_n257), .b(new_n259), .out0(\s[26] ));
  xnrc02aa1n02x5               g165(.a(\b[14] ), .b(\a[15] ), .out0(new_n261));
  xnrc02aa1n02x5               g166(.a(\b[15] ), .b(\a[16] ), .out0(new_n262));
  norp03aa1n02x5               g167(.a(new_n262), .b(new_n261), .c(new_n165), .o1(new_n263));
  nona32aa1n06x5               g168(.a(new_n154), .b(new_n262), .c(new_n164), .d(new_n261), .out0(new_n264));
  nona32aa1n03x5               g169(.a(new_n264), .b(new_n182), .c(new_n263), .d(new_n179), .out0(new_n265));
  and002aa1n06x5               g170(.a(new_n256), .b(new_n251), .o(new_n266));
  nano22aa1n06x5               g171(.a(new_n230), .b(new_n242), .c(new_n266), .out0(new_n267));
  aoai13aa1n06x5               g172(.a(new_n267), .b(new_n265), .c(new_n132), .d(new_n177), .o1(new_n268));
  oao003aa1n02x5               g173(.a(\a[26] ), .b(\b[25] ), .c(new_n258), .carry(new_n269));
  aobi12aa1n06x5               g174(.a(new_n269), .b(new_n250), .c(new_n266), .out0(new_n270));
  norp02aa1n02x5               g175(.a(\b[26] ), .b(\a[27] ), .o1(new_n271));
  nanp02aa1n02x5               g176(.a(\b[26] ), .b(\a[27] ), .o1(new_n272));
  norb02aa1n02x5               g177(.a(new_n272), .b(new_n271), .out0(new_n273));
  xnbna2aa1n03x5               g178(.a(new_n273), .b(new_n270), .c(new_n268), .out0(\s[27] ));
  inv000aa1n06x5               g179(.a(new_n271), .o1(new_n275));
  xnrc02aa1n02x5               g180(.a(\b[27] ), .b(\a[28] ), .out0(new_n276));
  inv020aa1n03x5               g181(.a(new_n267), .o1(new_n277));
  aoi012aa1n06x5               g182(.a(new_n277), .b(new_n178), .c(new_n187), .o1(new_n278));
  aoai13aa1n06x5               g183(.a(new_n242), .b(new_n232), .c(new_n217), .d(new_n229), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n266), .o1(new_n280));
  aoai13aa1n06x5               g185(.a(new_n269), .b(new_n280), .c(new_n279), .d(new_n249), .o1(new_n281));
  oai012aa1n02x5               g186(.a(new_n272), .b(new_n281), .c(new_n278), .o1(new_n282));
  aoi012aa1n02x7               g187(.a(new_n276), .b(new_n282), .c(new_n275), .o1(new_n283));
  aobi12aa1n03x5               g188(.a(new_n272), .b(new_n270), .c(new_n268), .out0(new_n284));
  nano22aa1n03x5               g189(.a(new_n284), .b(new_n275), .c(new_n276), .out0(new_n285));
  norp02aa1n03x5               g190(.a(new_n283), .b(new_n285), .o1(\s[28] ));
  nano22aa1n02x4               g191(.a(new_n276), .b(new_n275), .c(new_n272), .out0(new_n287));
  oai012aa1n02x5               g192(.a(new_n287), .b(new_n281), .c(new_n278), .o1(new_n288));
  oao003aa1n02x5               g193(.a(\a[28] ), .b(\b[27] ), .c(new_n275), .carry(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[28] ), .b(\a[29] ), .out0(new_n290));
  tech160nm_fiaoi012aa1n02p5x5 g195(.a(new_n290), .b(new_n288), .c(new_n289), .o1(new_n291));
  aobi12aa1n03x5               g196(.a(new_n287), .b(new_n270), .c(new_n268), .out0(new_n292));
  nano22aa1n03x5               g197(.a(new_n292), .b(new_n289), .c(new_n290), .out0(new_n293));
  norp02aa1n03x5               g198(.a(new_n291), .b(new_n293), .o1(\s[29] ));
  xnrb03aa1n02x5               g199(.a(new_n127), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g200(.a(new_n273), .b(new_n290), .c(new_n276), .out0(new_n296));
  oai012aa1n02x5               g201(.a(new_n296), .b(new_n281), .c(new_n278), .o1(new_n297));
  oao003aa1n02x5               g202(.a(\a[29] ), .b(\b[28] ), .c(new_n289), .carry(new_n298));
  xnrc02aa1n02x5               g203(.a(\b[29] ), .b(\a[30] ), .out0(new_n299));
  tech160nm_fiaoi012aa1n05x5   g204(.a(new_n299), .b(new_n297), .c(new_n298), .o1(new_n300));
  aobi12aa1n02x7               g205(.a(new_n296), .b(new_n270), .c(new_n268), .out0(new_n301));
  nano22aa1n03x5               g206(.a(new_n301), .b(new_n298), .c(new_n299), .out0(new_n302));
  norp02aa1n03x5               g207(.a(new_n300), .b(new_n302), .o1(\s[30] ));
  norb03aa1n02x5               g208(.a(new_n287), .b(new_n299), .c(new_n290), .out0(new_n304));
  aobi12aa1n03x5               g209(.a(new_n304), .b(new_n270), .c(new_n268), .out0(new_n305));
  oao003aa1n02x5               g210(.a(\a[30] ), .b(\b[29] ), .c(new_n298), .carry(new_n306));
  xnrc02aa1n02x5               g211(.a(\b[30] ), .b(\a[31] ), .out0(new_n307));
  nano22aa1n03x5               g212(.a(new_n305), .b(new_n306), .c(new_n307), .out0(new_n308));
  oai012aa1n02x5               g213(.a(new_n304), .b(new_n281), .c(new_n278), .o1(new_n309));
  tech160nm_fiaoi012aa1n02p5x5 g214(.a(new_n307), .b(new_n309), .c(new_n306), .o1(new_n310));
  norp02aa1n03x5               g215(.a(new_n310), .b(new_n308), .o1(\s[31] ));
  xnrb03aa1n02x5               g216(.a(new_n101), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g217(.a(\a[3] ), .b(\b[2] ), .c(new_n101), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g219(.a(new_n130), .b(\b[4] ), .c(new_n117), .out0(\s[5] ));
  nanp02aa1n02x5               g220(.a(new_n130), .b(new_n112), .o1(new_n316));
  oai112aa1n02x5               g221(.a(new_n316), .b(new_n111), .c(new_n118), .d(new_n117), .o1(new_n317));
  oaoi13aa1n02x5               g222(.a(new_n111), .b(new_n316), .c(new_n117), .d(new_n118), .o1(new_n318));
  norb02aa1n02x5               g223(.a(new_n317), .b(new_n318), .out0(\s[6] ));
  nanb02aa1n02x5               g224(.a(new_n106), .b(new_n107), .out0(new_n320));
  oaoi13aa1n04x5               g225(.a(new_n320), .b(new_n317), .c(\a[6] ), .d(\b[5] ), .o1(new_n321));
  oai112aa1n02x5               g226(.a(new_n317), .b(new_n320), .c(\b[5] ), .d(\a[6] ), .o1(new_n322));
  norb02aa1n02x5               g227(.a(new_n322), .b(new_n321), .out0(\s[7] ));
  norp02aa1n02x5               g228(.a(new_n321), .b(new_n106), .o1(new_n324));
  xnrb03aa1n02x5               g229(.a(new_n324), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xobna2aa1n03x5               g230(.a(new_n122), .b(new_n114), .c(new_n121), .out0(\s[9] ));
endmodule


