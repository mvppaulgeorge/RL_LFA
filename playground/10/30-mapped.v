// Benchmark "adder" written by ABC on Wed Jul 17 17:20:16 2024

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
    new_n132, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n166, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n271, new_n272,
    new_n273, new_n274, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n289, new_n290, new_n291, new_n292, new_n293, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n307, new_n308, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n317, new_n318, new_n319,
    new_n320, new_n321, new_n322, new_n323, new_n324, new_n325, new_n326,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n336, new_n337, new_n338, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n346, new_n347, new_n348, new_n349, new_n351,
    new_n352, new_n354, new_n355, new_n357, new_n358, new_n360, new_n361,
    new_n363, new_n364, new_n366, new_n367, new_n369;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n16x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\a[4] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\b[3] ), .o1(new_n100));
  nor042aa1n02x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  oaoi03aa1n02x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  nor042aa1n02x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  nand42aa1n02x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  nand02aa1n02x5               g009(.a(\b[0] ), .b(\a[1] ), .o1(new_n105));
  oai122aa1n03x5               g010(.a(new_n104), .b(new_n103), .c(new_n105), .d(\b[3] ), .e(\a[4] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[3] ), .b(\a[4] ), .o1(new_n107));
  nand42aa1n02x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nanb03aa1n03x5               g013(.a(new_n101), .b(new_n108), .c(new_n107), .out0(new_n109));
  tech160nm_fioai012aa1n05x5   g014(.a(new_n102), .b(new_n106), .c(new_n109), .o1(new_n110));
  aoi022aa1d24x5               g015(.a(\b[6] ), .b(\a[7] ), .c(\a[5] ), .d(\b[4] ), .o1(new_n111));
  oai122aa1n02x7               g016(.a(new_n111), .b(\a[6] ), .c(\b[5] ), .d(\a[5] ), .e(\b[4] ), .o1(new_n112));
  xnrc02aa1n02x5               g017(.a(\b[7] ), .b(\a[8] ), .out0(new_n113));
  nanp02aa1n24x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  oai012aa1n06x5               g019(.a(new_n114), .b(\b[6] ), .c(\a[7] ), .o1(new_n115));
  norp03aa1n06x5               g020(.a(new_n112), .b(new_n113), .c(new_n115), .o1(new_n116));
  oai022aa1n02x5               g021(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n117));
  tech160nm_fixorc02aa1n02p5x5 g022(.a(\a[8] ), .b(\b[7] ), .out0(new_n118));
  and002aa1n02x5               g023(.a(\b[6] ), .b(\a[7] ), .o(new_n119));
  nor042aa1n03x5               g024(.a(new_n115), .b(new_n119), .o1(new_n120));
  nand03aa1n02x5               g025(.a(new_n120), .b(new_n118), .c(new_n117), .o1(new_n121));
  inv000aa1d42x5               g026(.a(\a[8] ), .o1(new_n122));
  inv000aa1d42x5               g027(.a(\b[7] ), .o1(new_n123));
  nor042aa1n03x5               g028(.a(\b[6] ), .b(\a[7] ), .o1(new_n124));
  oaoi03aa1n09x5               g029(.a(new_n122), .b(new_n123), .c(new_n124), .o1(new_n125));
  nanp02aa1n02x5               g030(.a(new_n121), .b(new_n125), .o1(new_n126));
  xorc02aa1n12x5               g031(.a(\a[9] ), .b(\b[8] ), .out0(new_n127));
  aoai13aa1n02x5               g032(.a(new_n127), .b(new_n126), .c(new_n110), .d(new_n116), .o1(new_n128));
  nor002aa1n06x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nand42aa1n04x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nanb02aa1d24x5               g035(.a(new_n129), .b(new_n130), .out0(new_n131));
  inv000aa1d42x5               g036(.a(new_n131), .o1(new_n132));
  xnbna2aa1n03x5               g037(.a(new_n132), .b(new_n128), .c(new_n98), .out0(\s[10] ));
  nand42aa1n02x5               g038(.a(new_n110), .b(new_n116), .o1(new_n134));
  inv000aa1d42x5               g039(.a(new_n125), .o1(new_n135));
  aoi013aa1n06x4               g040(.a(new_n135), .b(new_n120), .c(new_n118), .d(new_n117), .o1(new_n136));
  nand02aa1d06x5               g041(.a(new_n134), .b(new_n136), .o1(new_n137));
  aoai13aa1n02x5               g042(.a(new_n132), .b(new_n97), .c(new_n137), .d(new_n127), .o1(new_n138));
  oaih12aa1n06x5               g043(.a(new_n130), .b(new_n129), .c(new_n97), .o1(new_n139));
  xnrc02aa1n12x5               g044(.a(\b[10] ), .b(\a[11] ), .out0(new_n140));
  inv000aa1d42x5               g045(.a(new_n140), .o1(new_n141));
  xnbna2aa1n03x5               g046(.a(new_n141), .b(new_n138), .c(new_n139), .out0(\s[11] ));
  aoai13aa1n02x5               g047(.a(new_n139), .b(new_n131), .c(new_n128), .d(new_n98), .o1(new_n143));
  nor022aa1n04x5               g048(.a(\b[10] ), .b(\a[11] ), .o1(new_n144));
  xnrc02aa1n12x5               g049(.a(\b[11] ), .b(\a[12] ), .out0(new_n145));
  aoai13aa1n02x5               g050(.a(new_n145), .b(new_n144), .c(new_n143), .d(new_n141), .o1(new_n146));
  aoi112aa1n02x5               g051(.a(new_n144), .b(new_n145), .c(new_n143), .d(new_n141), .o1(new_n147));
  nanb02aa1n02x5               g052(.a(new_n147), .b(new_n146), .out0(\s[12] ));
  nona23aa1d18x5               g053(.a(new_n141), .b(new_n127), .c(new_n145), .d(new_n131), .out0(new_n149));
  inv000aa1n02x5               g054(.a(new_n149), .o1(new_n150));
  aoai13aa1n02x5               g055(.a(new_n150), .b(new_n126), .c(new_n110), .d(new_n116), .o1(new_n151));
  inv000aa1d42x5               g056(.a(\a[12] ), .o1(new_n152));
  inv000aa1d42x5               g057(.a(\b[11] ), .o1(new_n153));
  oaoi03aa1n09x5               g058(.a(new_n152), .b(new_n153), .c(new_n144), .o1(new_n154));
  oai013aa1d12x5               g059(.a(new_n154), .b(new_n140), .c(new_n145), .d(new_n139), .o1(new_n155));
  inv000aa1d42x5               g060(.a(new_n155), .o1(new_n156));
  aoai13aa1n02x5               g061(.a(new_n156), .b(new_n149), .c(new_n134), .d(new_n136), .o1(new_n157));
  nanp02aa1n04x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nor022aa1n06x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  norb02aa1n02x5               g064(.a(new_n158), .b(new_n159), .out0(new_n160));
  inv000aa1n02x5               g065(.a(new_n139), .o1(new_n161));
  nor002aa1n02x5               g066(.a(new_n145), .b(new_n140), .o1(new_n162));
  inv000aa1d42x5               g067(.a(new_n154), .o1(new_n163));
  aoi112aa1n02x5               g068(.a(new_n163), .b(new_n160), .c(new_n162), .d(new_n161), .o1(new_n164));
  aoi022aa1n02x5               g069(.a(new_n157), .b(new_n160), .c(new_n151), .d(new_n164), .o1(\s[13] ));
  aoi012aa1n02x5               g070(.a(new_n159), .b(new_n157), .c(new_n158), .o1(new_n166));
  xnrb03aa1n02x5               g071(.a(new_n166), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n04x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  nand42aa1n04x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  nona23aa1d18x5               g074(.a(new_n158), .b(new_n169), .c(new_n168), .d(new_n159), .out0(new_n170));
  inv000aa1d42x5               g075(.a(new_n170), .o1(new_n171));
  nanp02aa1n02x5               g076(.a(new_n157), .b(new_n171), .o1(new_n172));
  oai012aa1n02x5               g077(.a(new_n169), .b(new_n168), .c(new_n159), .o1(new_n173));
  xnrc02aa1n12x5               g078(.a(\b[14] ), .b(\a[15] ), .out0(new_n174));
  inv000aa1d42x5               g079(.a(new_n174), .o1(new_n175));
  xnbna2aa1n03x5               g080(.a(new_n175), .b(new_n172), .c(new_n173), .out0(\s[15] ));
  aoai13aa1n04x5               g081(.a(new_n173), .b(new_n170), .c(new_n151), .d(new_n156), .o1(new_n177));
  norp02aa1n02x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  tech160nm_fixnrc02aa1n05x5   g083(.a(\b[15] ), .b(\a[16] ), .out0(new_n179));
  aoai13aa1n02x5               g084(.a(new_n179), .b(new_n178), .c(new_n177), .d(new_n175), .o1(new_n180));
  aoi112aa1n02x5               g085(.a(new_n178), .b(new_n179), .c(new_n177), .d(new_n175), .o1(new_n181));
  nanb02aa1n02x5               g086(.a(new_n181), .b(new_n180), .out0(\s[16] ));
  nor043aa1n09x5               g087(.a(new_n170), .b(new_n174), .c(new_n179), .o1(new_n183));
  norb02aa1n06x5               g088(.a(new_n183), .b(new_n149), .out0(new_n184));
  nanp02aa1n06x5               g089(.a(new_n137), .b(new_n184), .o1(new_n185));
  nor002aa1d32x5               g090(.a(\b[16] ), .b(\a[17] ), .o1(new_n186));
  nanp02aa1n02x5               g091(.a(\b[16] ), .b(\a[17] ), .o1(new_n187));
  aoi012aa1n03x5               g092(.a(new_n126), .b(new_n110), .c(new_n116), .o1(new_n188));
  nanb02aa1n12x5               g093(.a(new_n149), .b(new_n183), .out0(new_n189));
  inv000aa1d42x5               g094(.a(\a[16] ), .o1(new_n190));
  inv000aa1d42x5               g095(.a(\b[15] ), .o1(new_n191));
  oao003aa1n02x5               g096(.a(new_n190), .b(new_n191), .c(new_n178), .carry(new_n192));
  nor043aa1n03x5               g097(.a(new_n179), .b(new_n174), .c(new_n173), .o1(new_n193));
  aoi112aa1n09x5               g098(.a(new_n193), .b(new_n192), .c(new_n155), .d(new_n183), .o1(new_n194));
  oai012aa1n12x5               g099(.a(new_n194), .b(new_n189), .c(new_n188), .o1(new_n195));
  oaib12aa1n02x5               g100(.a(new_n195), .b(new_n186), .c(new_n187), .out0(new_n196));
  aoai13aa1n06x5               g101(.a(new_n183), .b(new_n163), .c(new_n162), .d(new_n161), .o1(new_n197));
  aoi112aa1n02x5               g102(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n198));
  aoi112aa1n02x5               g103(.a(new_n198), .b(new_n186), .c(new_n190), .d(new_n191), .o1(new_n199));
  nano32aa1n02x4               g104(.a(new_n193), .b(new_n197), .c(new_n187), .d(new_n199), .out0(new_n200));
  aob012aa1n02x5               g105(.a(new_n196), .b(new_n200), .c(new_n185), .out0(\s[17] ));
  xnrc02aa1n12x5               g106(.a(\b[17] ), .b(\a[18] ), .out0(new_n202));
  norb02aa1n02x5               g107(.a(new_n199), .b(new_n193), .out0(new_n203));
  oai112aa1n02x5               g108(.a(new_n197), .b(new_n203), .c(new_n189), .d(new_n188), .o1(new_n204));
  nanp02aa1n02x5               g109(.a(new_n204), .b(new_n187), .o1(new_n205));
  norp02aa1n02x5               g110(.a(\b[17] ), .b(\a[18] ), .o1(new_n206));
  and002aa1n02x5               g111(.a(\b[17] ), .b(\a[18] ), .o(new_n207));
  aoi112aa1n02x5               g112(.a(new_n207), .b(new_n206), .c(\a[17] ), .d(\b[16] ), .o1(new_n208));
  aoi022aa1n02x5               g113(.a(new_n205), .b(new_n202), .c(new_n204), .d(new_n208), .o1(\s[18] ));
  inv000aa1d42x5               g114(.a(new_n186), .o1(new_n210));
  nano22aa1n06x5               g115(.a(new_n202), .b(new_n210), .c(new_n187), .out0(new_n211));
  inv000aa1d42x5               g116(.a(new_n211), .o1(new_n212));
  aoi112aa1n03x5               g117(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n213));
  nor022aa1n04x5               g118(.a(new_n213), .b(new_n206), .o1(new_n214));
  aoai13aa1n04x5               g119(.a(new_n214), .b(new_n212), .c(new_n185), .d(new_n194), .o1(new_n215));
  nand02aa1n03x5               g120(.a(\b[18] ), .b(\a[19] ), .o1(new_n216));
  nor042aa1n04x5               g121(.a(\b[18] ), .b(\a[19] ), .o1(new_n217));
  norb02aa1n02x5               g122(.a(new_n216), .b(new_n217), .out0(new_n218));
  oaoi03aa1n09x5               g123(.a(\a[18] ), .b(\b[17] ), .c(new_n210), .o1(new_n219));
  aoi112aa1n02x5               g124(.a(new_n218), .b(new_n219), .c(new_n195), .d(new_n211), .o1(new_n220));
  aoi012aa1n02x5               g125(.a(new_n220), .b(new_n215), .c(new_n218), .o1(\s[19] ));
  xnrc02aa1n02x5               g126(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1n06x5               g127(.a(\b[19] ), .b(\a[20] ), .o1(new_n223));
  nand22aa1n04x5               g128(.a(\b[19] ), .b(\a[20] ), .o1(new_n224));
  nanb02aa1n02x5               g129(.a(new_n223), .b(new_n224), .out0(new_n225));
  aoai13aa1n03x5               g130(.a(new_n225), .b(new_n217), .c(new_n215), .d(new_n218), .o1(new_n226));
  aoai13aa1n03x5               g131(.a(new_n218), .b(new_n219), .c(new_n195), .d(new_n211), .o1(new_n227));
  nona22aa1n02x4               g132(.a(new_n227), .b(new_n225), .c(new_n217), .out0(new_n228));
  nanp02aa1n02x5               g133(.a(new_n226), .b(new_n228), .o1(\s[20] ));
  nona22aa1n09x5               g134(.a(new_n197), .b(new_n193), .c(new_n192), .out0(new_n230));
  nona23aa1n09x5               g135(.a(new_n216), .b(new_n224), .c(new_n223), .d(new_n217), .out0(new_n231));
  nano23aa1n03x7               g136(.a(new_n231), .b(new_n202), .c(new_n187), .d(new_n210), .out0(new_n232));
  aoai13aa1n02x5               g137(.a(new_n232), .b(new_n230), .c(new_n137), .d(new_n184), .o1(new_n233));
  inv020aa1n02x5               g138(.a(new_n232), .o1(new_n234));
  tech160nm_fiaoi012aa1n04x5   g139(.a(new_n223), .b(new_n217), .c(new_n224), .o1(new_n235));
  oai012aa1n18x5               g140(.a(new_n235), .b(new_n231), .c(new_n214), .o1(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  aoai13aa1n04x5               g142(.a(new_n237), .b(new_n234), .c(new_n185), .d(new_n194), .o1(new_n238));
  xnrc02aa1n12x5               g143(.a(\b[20] ), .b(\a[21] ), .out0(new_n239));
  inv000aa1d42x5               g144(.a(new_n239), .o1(new_n240));
  nano23aa1n06x5               g145(.a(new_n223), .b(new_n217), .c(new_n224), .d(new_n216), .out0(new_n241));
  inv000aa1n06x5               g146(.a(new_n235), .o1(new_n242));
  aoi112aa1n02x5               g147(.a(new_n242), .b(new_n240), .c(new_n241), .d(new_n219), .o1(new_n243));
  aoi022aa1n02x5               g148(.a(new_n238), .b(new_n240), .c(new_n233), .d(new_n243), .o1(\s[21] ));
  nor042aa1n03x5               g149(.a(\b[20] ), .b(\a[21] ), .o1(new_n245));
  tech160nm_fixnrc02aa1n04x5   g150(.a(\b[21] ), .b(\a[22] ), .out0(new_n246));
  aoai13aa1n03x5               g151(.a(new_n246), .b(new_n245), .c(new_n238), .d(new_n240), .o1(new_n247));
  aoai13aa1n03x5               g152(.a(new_n240), .b(new_n236), .c(new_n195), .d(new_n232), .o1(new_n248));
  nona22aa1n02x4               g153(.a(new_n248), .b(new_n246), .c(new_n245), .out0(new_n249));
  nanp02aa1n02x5               g154(.a(new_n247), .b(new_n249), .o1(\s[22] ));
  nor042aa1d18x5               g155(.a(new_n246), .b(new_n239), .o1(new_n251));
  inv000aa1d42x5               g156(.a(new_n251), .o1(new_n252));
  nano22aa1n02x4               g157(.a(new_n252), .b(new_n211), .c(new_n241), .out0(new_n253));
  aoai13aa1n12x5               g158(.a(new_n251), .b(new_n242), .c(new_n241), .d(new_n219), .o1(new_n254));
  inv000aa1d42x5               g159(.a(\a[22] ), .o1(new_n255));
  inv000aa1d42x5               g160(.a(\b[21] ), .o1(new_n256));
  oaoi03aa1n09x5               g161(.a(new_n255), .b(new_n256), .c(new_n245), .o1(new_n257));
  nanp02aa1n02x5               g162(.a(new_n254), .b(new_n257), .o1(new_n258));
  xnrc02aa1n02x5               g163(.a(\b[22] ), .b(\a[23] ), .out0(new_n259));
  aoai13aa1n02x5               g164(.a(new_n259), .b(new_n258), .c(new_n195), .d(new_n253), .o1(new_n260));
  aoai13aa1n06x5               g165(.a(new_n253), .b(new_n230), .c(new_n137), .d(new_n184), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n254), .o1(new_n262));
  inv000aa1n02x5               g167(.a(new_n257), .o1(new_n263));
  nona32aa1n02x4               g168(.a(new_n261), .b(new_n259), .c(new_n263), .d(new_n262), .out0(new_n264));
  nanp02aa1n02x5               g169(.a(new_n260), .b(new_n264), .o1(\s[23] ));
  nor042aa1n02x5               g170(.a(\b[23] ), .b(\a[24] ), .o1(new_n266));
  nand42aa1n03x5               g171(.a(\b[23] ), .b(\a[24] ), .o1(new_n267));
  nanb02aa1n02x5               g172(.a(new_n266), .b(new_n267), .out0(new_n268));
  nor042aa1n02x5               g173(.a(\b[22] ), .b(\a[23] ), .o1(new_n269));
  nona32aa1n03x5               g174(.a(new_n261), .b(new_n269), .c(new_n263), .d(new_n262), .out0(new_n270));
  inv000aa1d42x5               g175(.a(\a[23] ), .o1(new_n271));
  oaib12aa1n02x5               g176(.a(new_n270), .b(new_n271), .c(\b[22] ), .out0(new_n272));
  nand42aa1n02x5               g177(.a(\b[22] ), .b(\a[23] ), .o1(new_n273));
  nano22aa1n02x4               g178(.a(new_n266), .b(new_n273), .c(new_n267), .out0(new_n274));
  aoi022aa1n02x5               g179(.a(new_n272), .b(new_n268), .c(new_n270), .d(new_n274), .o1(\s[24] ));
  nano23aa1d12x5               g180(.a(new_n269), .b(new_n266), .c(new_n267), .d(new_n273), .out0(new_n276));
  nano32aa1n03x7               g181(.a(new_n252), .b(new_n211), .c(new_n276), .d(new_n241), .out0(new_n277));
  aoai13aa1n02x5               g182(.a(new_n277), .b(new_n230), .c(new_n137), .d(new_n184), .o1(new_n278));
  inv000aa1n02x5               g183(.a(new_n277), .o1(new_n279));
  inv000aa1n02x5               g184(.a(new_n276), .o1(new_n280));
  oai012aa1n02x5               g185(.a(new_n267), .b(new_n266), .c(new_n269), .o1(new_n281));
  aoai13aa1n09x5               g186(.a(new_n281), .b(new_n280), .c(new_n254), .d(new_n257), .o1(new_n282));
  inv040aa1n03x5               g187(.a(new_n282), .o1(new_n283));
  aoai13aa1n06x5               g188(.a(new_n283), .b(new_n279), .c(new_n185), .d(new_n194), .o1(new_n284));
  xorc02aa1n12x5               g189(.a(\a[25] ), .b(\b[24] ), .out0(new_n285));
  aoai13aa1n06x5               g190(.a(new_n276), .b(new_n263), .c(new_n236), .d(new_n251), .o1(new_n286));
  nano22aa1n02x4               g191(.a(new_n285), .b(new_n286), .c(new_n281), .out0(new_n287));
  aoi022aa1n02x5               g192(.a(new_n284), .b(new_n285), .c(new_n278), .d(new_n287), .o1(\s[25] ));
  norp02aa1n02x5               g193(.a(\b[24] ), .b(\a[25] ), .o1(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[25] ), .b(\a[26] ), .out0(new_n290));
  aoai13aa1n03x5               g195(.a(new_n290), .b(new_n289), .c(new_n284), .d(new_n285), .o1(new_n291));
  aoai13aa1n02x5               g196(.a(new_n285), .b(new_n282), .c(new_n195), .d(new_n277), .o1(new_n292));
  nona22aa1n02x4               g197(.a(new_n292), .b(new_n290), .c(new_n289), .out0(new_n293));
  nanp02aa1n03x5               g198(.a(new_n291), .b(new_n293), .o1(\s[26] ));
  norb02aa1n06x5               g199(.a(new_n285), .b(new_n290), .out0(new_n295));
  nano32aa1n03x7               g200(.a(new_n234), .b(new_n295), .c(new_n251), .d(new_n276), .out0(new_n296));
  aoai13aa1n06x5               g201(.a(new_n296), .b(new_n230), .c(new_n137), .d(new_n184), .o1(new_n297));
  inv000aa1d42x5               g202(.a(\a[26] ), .o1(new_n298));
  inv000aa1d42x5               g203(.a(\b[25] ), .o1(new_n299));
  tech160nm_fioaoi03aa1n03p5x5 g204(.a(new_n298), .b(new_n299), .c(new_n289), .o1(new_n300));
  inv000aa1d42x5               g205(.a(new_n300), .o1(new_n301));
  tech160nm_fiaoi012aa1n05x5   g206(.a(new_n301), .b(new_n282), .c(new_n295), .o1(new_n302));
  norp02aa1n02x5               g207(.a(\b[26] ), .b(\a[27] ), .o1(new_n303));
  nand42aa1n03x5               g208(.a(\b[26] ), .b(\a[27] ), .o1(new_n304));
  norb02aa1n06x4               g209(.a(new_n304), .b(new_n303), .out0(new_n305));
  nanp02aa1n03x5               g210(.a(new_n282), .b(new_n295), .o1(new_n306));
  and002aa1n02x5               g211(.a(new_n300), .b(new_n305), .o(new_n307));
  nanp03aa1n02x5               g212(.a(new_n297), .b(new_n306), .c(new_n307), .o1(new_n308));
  aoai13aa1n02x5               g213(.a(new_n308), .b(new_n305), .c(new_n297), .d(new_n302), .o1(\s[27] ));
  xnrc02aa1n02x5               g214(.a(\b[27] ), .b(\a[28] ), .out0(new_n310));
  norb02aa1n02x5               g215(.a(new_n300), .b(new_n303), .out0(new_n311));
  nand23aa1n03x5               g216(.a(new_n297), .b(new_n306), .c(new_n311), .o1(new_n312));
  nanp02aa1n03x5               g217(.a(new_n312), .b(new_n304), .o1(new_n313));
  oai012aa1n02x5               g218(.a(new_n304), .b(\b[27] ), .c(\a[28] ), .o1(new_n314));
  aoi012aa1n02x5               g219(.a(new_n314), .b(\a[28] ), .c(\b[27] ), .o1(new_n315));
  aoi022aa1n02x7               g220(.a(new_n313), .b(new_n310), .c(new_n312), .d(new_n315), .o1(\s[28] ));
  inv000aa1d42x5               g221(.a(new_n295), .o1(new_n317));
  aoai13aa1n02x7               g222(.a(new_n300), .b(new_n317), .c(new_n286), .d(new_n281), .o1(new_n318));
  norb02aa1n03x5               g223(.a(new_n305), .b(new_n310), .out0(new_n319));
  aoai13aa1n02x7               g224(.a(new_n319), .b(new_n318), .c(new_n195), .d(new_n296), .o1(new_n320));
  inv000aa1n02x5               g225(.a(new_n319), .o1(new_n321));
  orn002aa1n03x5               g226(.a(\a[27] ), .b(\b[26] ), .o(new_n322));
  oao003aa1n03x5               g227(.a(\a[28] ), .b(\b[27] ), .c(new_n322), .carry(new_n323));
  aoai13aa1n03x5               g228(.a(new_n323), .b(new_n321), .c(new_n302), .d(new_n297), .o1(new_n324));
  xorc02aa1n02x5               g229(.a(\a[29] ), .b(\b[28] ), .out0(new_n325));
  norb02aa1n02x5               g230(.a(new_n323), .b(new_n325), .out0(new_n326));
  aoi022aa1n03x5               g231(.a(new_n324), .b(new_n325), .c(new_n320), .d(new_n326), .o1(\s[29] ));
  xorb03aa1n02x5               g232(.a(new_n105), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x5               g233(.a(new_n310), .b(new_n325), .c(new_n305), .out0(new_n329));
  aoai13aa1n03x5               g234(.a(new_n329), .b(new_n318), .c(new_n195), .d(new_n296), .o1(new_n330));
  inv000aa1d42x5               g235(.a(new_n329), .o1(new_n331));
  tech160nm_fioaoi03aa1n02p5x5 g236(.a(\a[29] ), .b(\b[28] ), .c(new_n323), .o1(new_n332));
  inv000aa1d42x5               g237(.a(new_n332), .o1(new_n333));
  aoai13aa1n03x5               g238(.a(new_n333), .b(new_n331), .c(new_n302), .d(new_n297), .o1(new_n334));
  xorc02aa1n02x5               g239(.a(\a[30] ), .b(\b[29] ), .out0(new_n335));
  aoi012aa1n02x5               g240(.a(new_n323), .b(\a[29] ), .c(\b[28] ), .o1(new_n336));
  oabi12aa1n02x5               g241(.a(new_n335), .b(\a[29] ), .c(\b[28] ), .out0(new_n337));
  norp02aa1n02x5               g242(.a(new_n336), .b(new_n337), .o1(new_n338));
  aoi022aa1n03x5               g243(.a(new_n334), .b(new_n335), .c(new_n330), .d(new_n338), .o1(\s[30] ));
  nano22aa1n02x5               g244(.a(new_n321), .b(new_n325), .c(new_n335), .out0(new_n340));
  aoai13aa1n02x5               g245(.a(new_n340), .b(new_n318), .c(new_n195), .d(new_n296), .o1(new_n341));
  xorc02aa1n02x5               g246(.a(\a[31] ), .b(\b[30] ), .out0(new_n342));
  inv000aa1d42x5               g247(.a(\a[30] ), .o1(new_n343));
  inv000aa1d42x5               g248(.a(\b[29] ), .o1(new_n344));
  oabi12aa1n02x5               g249(.a(new_n342), .b(\a[30] ), .c(\b[29] ), .out0(new_n345));
  oaoi13aa1n04x5               g250(.a(new_n345), .b(new_n332), .c(new_n343), .d(new_n344), .o1(new_n346));
  inv000aa1d42x5               g251(.a(new_n340), .o1(new_n347));
  oaoi03aa1n03x5               g252(.a(new_n343), .b(new_n344), .c(new_n332), .o1(new_n348));
  aoai13aa1n03x5               g253(.a(new_n348), .b(new_n347), .c(new_n302), .d(new_n297), .o1(new_n349));
  aoi022aa1n03x5               g254(.a(new_n349), .b(new_n342), .c(new_n341), .d(new_n346), .o1(\s[31] ));
  nona22aa1n02x4               g255(.a(new_n104), .b(new_n103), .c(new_n105), .out0(new_n351));
  norb02aa1n02x5               g256(.a(new_n108), .b(new_n101), .out0(new_n352));
  xobna2aa1n03x5               g257(.a(new_n352), .b(new_n351), .c(new_n104), .out0(\s[3] ));
  aob012aa1n02x5               g258(.a(new_n352), .b(new_n351), .c(new_n104), .out0(new_n354));
  xnrc02aa1n02x5               g259(.a(\b[3] ), .b(\a[4] ), .out0(new_n355));
  xnbna2aa1n03x5               g260(.a(new_n355), .b(new_n354), .c(new_n108), .out0(\s[4] ));
  xorc02aa1n02x5               g261(.a(\a[5] ), .b(\b[4] ), .out0(new_n357));
  oai112aa1n02x5               g262(.a(new_n102), .b(new_n357), .c(new_n106), .d(new_n109), .o1(new_n358));
  oaib12aa1n02x5               g263(.a(new_n358), .b(new_n357), .c(new_n110), .out0(\s[5] ));
  nanp02aa1n02x5               g264(.a(\b[4] ), .b(\a[5] ), .o1(new_n360));
  xnrc02aa1n02x5               g265(.a(\b[5] ), .b(\a[6] ), .out0(new_n361));
  xnbna2aa1n03x5               g266(.a(new_n361), .b(new_n358), .c(new_n360), .out0(\s[6] ));
  tech160nm_fiao0012aa1n02p5x5 g267(.a(new_n361), .b(new_n358), .c(new_n360), .o(new_n363));
  norp02aa1n02x5               g268(.a(new_n119), .b(new_n124), .o1(new_n364));
  xobna2aa1n03x5               g269(.a(new_n364), .b(new_n363), .c(new_n114), .out0(\s[7] ));
  aoai13aa1n02x5               g270(.a(new_n114), .b(new_n361), .c(new_n358), .d(new_n360), .o1(new_n366));
  oaoi03aa1n02x5               g271(.a(\a[7] ), .b(\b[6] ), .c(new_n366), .o1(new_n367));
  xorb03aa1n02x5               g272(.a(new_n367), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  aoi113aa1n02x5               g273(.a(new_n135), .b(new_n127), .c(new_n120), .d(new_n118), .e(new_n117), .o1(new_n369));
  aoi022aa1n02x5               g274(.a(new_n137), .b(new_n127), .c(new_n134), .d(new_n369), .o1(\s[9] ));
endmodule


