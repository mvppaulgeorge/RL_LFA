// Benchmark "adder" written by ABC on Thu Jul 18 05:58:32 2024

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
    new_n133, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n155, new_n156,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n189, new_n190, new_n191, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n326, new_n329, new_n332, new_n333,
    new_n335, new_n336, new_n337, new_n339;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\b[8] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  nand42aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand22aa1n09x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  nor042aa1n04x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  oai012aa1n12x5               g007(.a(new_n100), .b(new_n102), .c(new_n101), .o1(new_n103));
  xnrc02aa1n12x5               g008(.a(\b[3] ), .b(\a[4] ), .out0(new_n104));
  norp02aa1n04x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand42aa1n03x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nanb02aa1n06x5               g011(.a(new_n105), .b(new_n106), .out0(new_n107));
  oai022aa1n02x7               g012(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n108));
  aob012aa1n06x5               g013(.a(new_n108), .b(\b[3] ), .c(\a[4] ), .out0(new_n109));
  oai013aa1d12x5               g014(.a(new_n109), .b(new_n104), .c(new_n103), .d(new_n107), .o1(new_n110));
  tech160nm_finand02aa1n03p5x5 g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nor022aa1n16x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nanp02aa1n04x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  norp02aa1n24x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nona23aa1n09x5               g019(.a(new_n113), .b(new_n111), .c(new_n114), .d(new_n112), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  xnrc02aa1n03x5               g021(.a(\b[5] ), .b(\a[6] ), .out0(new_n117));
  norp03aa1d12x5               g022(.a(new_n115), .b(new_n116), .c(new_n117), .o1(new_n118));
  tech160nm_fioai012aa1n03p5x5 g023(.a(new_n113), .b(new_n114), .c(new_n112), .o1(new_n119));
  inv000aa1d42x5               g024(.a(\a[6] ), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\b[5] ), .o1(new_n121));
  nor042aa1n02x5               g026(.a(\b[4] ), .b(\a[5] ), .o1(new_n122));
  oaoi03aa1n02x5               g027(.a(new_n120), .b(new_n121), .c(new_n122), .o1(new_n123));
  tech160nm_fioai012aa1n05x5   g028(.a(new_n119), .b(new_n115), .c(new_n123), .o1(new_n124));
  xorc02aa1n02x5               g029(.a(\a[9] ), .b(\b[8] ), .out0(new_n125));
  aoai13aa1n06x5               g030(.a(new_n125), .b(new_n124), .c(new_n110), .d(new_n118), .o1(new_n126));
  nor002aa1d32x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nand02aa1d28x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  norb02aa1n02x5               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n129), .b(new_n126), .c(new_n99), .out0(\s[10] ));
  inv000aa1d42x5               g035(.a(new_n127), .o1(new_n131));
  inv000aa1n03x5               g036(.a(new_n128), .o1(new_n132));
  aoai13aa1n06x5               g037(.a(new_n131), .b(new_n132), .c(new_n126), .d(new_n99), .o1(new_n133));
  xorb03aa1n02x5               g038(.a(new_n133), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1d24x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nanp02aa1n12x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nor002aa1d32x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nand42aa1n06x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(new_n139));
  aoi112aa1n02x5               g044(.a(new_n139), .b(new_n135), .c(new_n133), .d(new_n136), .o1(new_n140));
  aoai13aa1n03x5               g045(.a(new_n139), .b(new_n135), .c(new_n133), .d(new_n136), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(\s[12] ));
  and002aa1n18x5               g047(.a(\b[8] ), .b(\a[9] ), .o(new_n143));
  inv000aa1d42x5               g048(.a(new_n143), .o1(new_n144));
  nona23aa1n09x5               g049(.a(new_n138), .b(new_n136), .c(new_n135), .d(new_n137), .out0(new_n145));
  nano32aa1n02x4               g050(.a(new_n145), .b(new_n129), .c(new_n144), .d(new_n99), .out0(new_n146));
  aoai13aa1n06x5               g051(.a(new_n146), .b(new_n124), .c(new_n110), .d(new_n118), .o1(new_n147));
  tech160nm_fioai012aa1n03p5x5 g052(.a(new_n138), .b(new_n137), .c(new_n135), .o1(new_n148));
  aoai13aa1n12x5               g053(.a(new_n128), .b(new_n127), .c(new_n97), .d(new_n98), .o1(new_n149));
  oai012aa1d24x5               g054(.a(new_n148), .b(new_n145), .c(new_n149), .o1(new_n150));
  inv000aa1d42x5               g055(.a(new_n150), .o1(new_n151));
  nanp02aa1n02x5               g056(.a(new_n147), .b(new_n151), .o1(new_n152));
  xorb03aa1n02x5               g057(.a(new_n152), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g058(.a(\a[13] ), .o1(new_n154));
  inv000aa1d42x5               g059(.a(\b[12] ), .o1(new_n155));
  oaoi03aa1n02x5               g060(.a(new_n154), .b(new_n155), .c(new_n152), .o1(new_n156));
  xnrb03aa1n02x5               g061(.a(new_n156), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor022aa1n08x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  tech160nm_finand02aa1n03p5x5 g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  aoai13aa1n02x5               g064(.a(new_n159), .b(new_n158), .c(new_n154), .d(new_n155), .o1(new_n160));
  norp02aa1n04x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  nanp02aa1n09x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  nona23aa1n03x5               g067(.a(new_n159), .b(new_n162), .c(new_n161), .d(new_n158), .out0(new_n163));
  aoai13aa1n06x5               g068(.a(new_n160), .b(new_n163), .c(new_n147), .d(new_n151), .o1(new_n164));
  xorb03aa1n02x5               g069(.a(new_n164), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n04x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  nand22aa1n06x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  nor042aa1n04x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  nand22aa1n04x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nanb02aa1n02x5               g074(.a(new_n168), .b(new_n169), .out0(new_n170));
  inv000aa1d42x5               g075(.a(new_n170), .o1(new_n171));
  aoi112aa1n02x5               g076(.a(new_n171), .b(new_n166), .c(new_n164), .d(new_n167), .o1(new_n172));
  aoai13aa1n03x5               g077(.a(new_n171), .b(new_n166), .c(new_n164), .d(new_n167), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n173), .b(new_n172), .out0(\s[16] ));
  aoi112aa1n02x7               g079(.a(new_n132), .b(new_n127), .c(new_n97), .d(new_n98), .o1(new_n175));
  nano23aa1n02x5               g080(.a(new_n135), .b(new_n137), .c(new_n138), .d(new_n136), .out0(new_n176));
  nano23aa1n02x5               g081(.a(new_n161), .b(new_n158), .c(new_n159), .d(new_n162), .out0(new_n177));
  nano23aa1n06x5               g082(.a(new_n166), .b(new_n168), .c(new_n169), .d(new_n167), .out0(new_n178));
  nand02aa1n02x5               g083(.a(new_n178), .b(new_n177), .o1(new_n179));
  nano32aa1n03x7               g084(.a(new_n179), .b(new_n176), .c(new_n175), .d(new_n144), .out0(new_n180));
  aoai13aa1n12x5               g085(.a(new_n180), .b(new_n124), .c(new_n110), .d(new_n118), .o1(new_n181));
  nona23aa1n09x5               g086(.a(new_n169), .b(new_n167), .c(new_n166), .d(new_n168), .out0(new_n182));
  norp02aa1n04x5               g087(.a(new_n182), .b(new_n163), .o1(new_n183));
  oa0012aa1n02x5               g088(.a(new_n169), .b(new_n168), .c(new_n166), .o(new_n184));
  nor002aa1n02x5               g089(.a(new_n182), .b(new_n160), .o1(new_n185));
  aoi112aa1n09x5               g090(.a(new_n185), .b(new_n184), .c(new_n150), .d(new_n183), .o1(new_n186));
  nand02aa1d06x5               g091(.a(new_n181), .b(new_n186), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g093(.a(\a[17] ), .o1(new_n189));
  inv000aa1d42x5               g094(.a(\b[16] ), .o1(new_n190));
  oaoi03aa1n03x5               g095(.a(new_n189), .b(new_n190), .c(new_n187), .o1(new_n191));
  xnrb03aa1n03x5               g096(.a(new_n191), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nanp02aa1n02x5               g097(.a(new_n190), .b(new_n189), .o1(new_n193));
  nanp02aa1n02x5               g098(.a(\b[16] ), .b(\a[17] ), .o1(new_n194));
  nor022aa1n06x5               g099(.a(\b[17] ), .b(\a[18] ), .o1(new_n195));
  nand42aa1n03x5               g100(.a(\b[17] ), .b(\a[18] ), .o1(new_n196));
  nanb02aa1n03x5               g101(.a(new_n195), .b(new_n196), .out0(new_n197));
  nano22aa1d15x5               g102(.a(new_n197), .b(new_n193), .c(new_n194), .out0(new_n198));
  inv000aa1d42x5               g103(.a(new_n198), .o1(new_n199));
  aoai13aa1n06x5               g104(.a(new_n196), .b(new_n195), .c(new_n189), .d(new_n190), .o1(new_n200));
  aoai13aa1n06x5               g105(.a(new_n200), .b(new_n199), .c(new_n181), .d(new_n186), .o1(new_n201));
  xorb03aa1n02x5               g106(.a(new_n201), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g107(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n09x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  nand22aa1n03x5               g109(.a(\b[18] ), .b(\a[19] ), .o1(new_n205));
  nor042aa1n02x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  nanp02aa1n03x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  nanb02aa1n02x5               g112(.a(new_n206), .b(new_n207), .out0(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  aoi112aa1n03x4               g114(.a(new_n204), .b(new_n209), .c(new_n201), .d(new_n205), .o1(new_n210));
  inv000aa1d42x5               g115(.a(new_n204), .o1(new_n211));
  norb02aa1n02x5               g116(.a(new_n205), .b(new_n204), .out0(new_n212));
  nanp02aa1n03x5               g117(.a(new_n201), .b(new_n212), .o1(new_n213));
  tech160nm_fiaoi012aa1n02p5x5 g118(.a(new_n208), .b(new_n213), .c(new_n211), .o1(new_n214));
  norp02aa1n03x5               g119(.a(new_n214), .b(new_n210), .o1(\s[20] ));
  nano23aa1n06x5               g120(.a(new_n204), .b(new_n206), .c(new_n207), .d(new_n205), .out0(new_n216));
  nanp02aa1n02x5               g121(.a(new_n198), .b(new_n216), .o1(new_n217));
  inv000aa1n02x5               g122(.a(new_n200), .o1(new_n218));
  tech160nm_fioai012aa1n04x5   g123(.a(new_n207), .b(new_n206), .c(new_n204), .o1(new_n219));
  aobi12aa1n06x5               g124(.a(new_n219), .b(new_n216), .c(new_n218), .out0(new_n220));
  aoai13aa1n06x5               g125(.a(new_n220), .b(new_n217), .c(new_n181), .d(new_n186), .o1(new_n221));
  xorb03aa1n02x5               g126(.a(new_n221), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n06x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  nanp02aa1n02x5               g128(.a(\b[20] ), .b(\a[21] ), .o1(new_n224));
  xorc02aa1n02x5               g129(.a(\a[22] ), .b(\b[21] ), .out0(new_n225));
  aoi112aa1n03x4               g130(.a(new_n223), .b(new_n225), .c(new_n221), .d(new_n224), .o1(new_n226));
  inv000aa1n02x5               g131(.a(new_n223), .o1(new_n227));
  norb02aa1n02x5               g132(.a(new_n224), .b(new_n223), .out0(new_n228));
  nanp02aa1n03x5               g133(.a(new_n221), .b(new_n228), .o1(new_n229));
  inv000aa1d42x5               g134(.a(new_n225), .o1(new_n230));
  tech160nm_fiaoi012aa1n03p5x5 g135(.a(new_n230), .b(new_n229), .c(new_n227), .o1(new_n231));
  nor042aa1n03x5               g136(.a(new_n231), .b(new_n226), .o1(\s[22] ));
  norp02aa1n02x5               g137(.a(\b[21] ), .b(\a[22] ), .o1(new_n233));
  nanp02aa1n02x5               g138(.a(\b[21] ), .b(\a[22] ), .o1(new_n234));
  nano23aa1n06x5               g139(.a(new_n223), .b(new_n233), .c(new_n234), .d(new_n224), .out0(new_n235));
  nand23aa1n03x5               g140(.a(new_n198), .b(new_n216), .c(new_n235), .o1(new_n236));
  nona23aa1n03x5               g141(.a(new_n207), .b(new_n205), .c(new_n204), .d(new_n206), .out0(new_n237));
  tech160nm_fioai012aa1n05x5   g142(.a(new_n219), .b(new_n237), .c(new_n200), .o1(new_n238));
  tech160nm_fioaoi03aa1n02p5x5 g143(.a(\a[22] ), .b(\b[21] ), .c(new_n227), .o1(new_n239));
  aoi012aa1n02x5               g144(.a(new_n239), .b(new_n238), .c(new_n235), .o1(new_n240));
  aoai13aa1n06x5               g145(.a(new_n240), .b(new_n236), .c(new_n181), .d(new_n186), .o1(new_n241));
  xorb03aa1n02x5               g146(.a(new_n241), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor022aa1n16x5               g147(.a(\b[22] ), .b(\a[23] ), .o1(new_n243));
  nanp02aa1n02x5               g148(.a(\b[22] ), .b(\a[23] ), .o1(new_n244));
  nor002aa1n03x5               g149(.a(\b[23] ), .b(\a[24] ), .o1(new_n245));
  nanp02aa1n04x5               g150(.a(\b[23] ), .b(\a[24] ), .o1(new_n246));
  norb02aa1n02x5               g151(.a(new_n246), .b(new_n245), .out0(new_n247));
  aoi112aa1n03x4               g152(.a(new_n243), .b(new_n247), .c(new_n241), .d(new_n244), .o1(new_n248));
  inv000aa1d42x5               g153(.a(new_n243), .o1(new_n249));
  norb02aa1n02x5               g154(.a(new_n244), .b(new_n243), .out0(new_n250));
  nanp02aa1n03x5               g155(.a(new_n241), .b(new_n250), .o1(new_n251));
  inv000aa1d42x5               g156(.a(new_n247), .o1(new_n252));
  tech160nm_fiaoi012aa1n03p5x5 g157(.a(new_n252), .b(new_n251), .c(new_n249), .o1(new_n253));
  nor042aa1n03x5               g158(.a(new_n253), .b(new_n248), .o1(\s[24] ));
  nano23aa1n09x5               g159(.a(new_n243), .b(new_n245), .c(new_n246), .d(new_n244), .out0(new_n255));
  nanb03aa1n02x5               g160(.a(new_n217), .b(new_n255), .c(new_n235), .out0(new_n256));
  nona22aa1n02x4               g161(.a(new_n246), .b(new_n245), .c(new_n243), .out0(new_n257));
  aoi022aa1n06x5               g162(.a(new_n255), .b(new_n239), .c(new_n257), .d(new_n246), .o1(new_n258));
  inv000aa1n06x5               g163(.a(new_n258), .o1(new_n259));
  aoi013aa1n03x5               g164(.a(new_n259), .b(new_n238), .c(new_n235), .d(new_n255), .o1(new_n260));
  aoai13aa1n06x5               g165(.a(new_n260), .b(new_n256), .c(new_n181), .d(new_n186), .o1(new_n261));
  xorb03aa1n02x5               g166(.a(new_n261), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n04x5               g167(.a(\b[24] ), .b(\a[25] ), .o1(new_n263));
  nanp02aa1n02x5               g168(.a(\b[24] ), .b(\a[25] ), .o1(new_n264));
  xnrc02aa1n12x5               g169(.a(\b[25] ), .b(\a[26] ), .out0(new_n265));
  inv000aa1d42x5               g170(.a(new_n265), .o1(new_n266));
  aoi112aa1n03x4               g171(.a(new_n263), .b(new_n266), .c(new_n261), .d(new_n264), .o1(new_n267));
  inv000aa1d42x5               g172(.a(new_n263), .o1(new_n268));
  norb02aa1n02x5               g173(.a(new_n264), .b(new_n263), .out0(new_n269));
  nand02aa1n02x5               g174(.a(new_n261), .b(new_n269), .o1(new_n270));
  tech160nm_fiaoi012aa1n02p5x5 g175(.a(new_n265), .b(new_n270), .c(new_n268), .o1(new_n271));
  norp02aa1n03x5               g176(.a(new_n271), .b(new_n267), .o1(\s[26] ));
  inv000aa1d42x5               g177(.a(new_n104), .o1(new_n273));
  nona22aa1n02x4               g178(.a(new_n273), .b(new_n103), .c(new_n107), .out0(new_n274));
  nano23aa1n02x5               g179(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n275));
  nona22aa1n02x4               g180(.a(new_n275), .b(new_n116), .c(new_n117), .out0(new_n276));
  oao003aa1n02x5               g181(.a(new_n120), .b(new_n121), .c(new_n122), .carry(new_n277));
  aobi12aa1n06x5               g182(.a(new_n119), .b(new_n275), .c(new_n277), .out0(new_n278));
  aoai13aa1n04x5               g183(.a(new_n278), .b(new_n276), .c(new_n274), .d(new_n109), .o1(new_n279));
  nanp02aa1n02x5               g184(.a(new_n150), .b(new_n183), .o1(new_n280));
  nona22aa1n02x4               g185(.a(new_n280), .b(new_n185), .c(new_n184), .out0(new_n281));
  nano22aa1n12x5               g186(.a(new_n265), .b(new_n268), .c(new_n264), .out0(new_n282));
  nano22aa1n03x7               g187(.a(new_n236), .b(new_n255), .c(new_n282), .out0(new_n283));
  aoai13aa1n06x5               g188(.a(new_n283), .b(new_n281), .c(new_n279), .d(new_n180), .o1(new_n284));
  nano22aa1n03x5               g189(.a(new_n220), .b(new_n235), .c(new_n255), .out0(new_n285));
  oaoi03aa1n02x5               g190(.a(\a[26] ), .b(\b[25] ), .c(new_n268), .o1(new_n286));
  oaoi13aa1n09x5               g191(.a(new_n286), .b(new_n282), .c(new_n285), .d(new_n259), .o1(new_n287));
  nor042aa1n06x5               g192(.a(\b[26] ), .b(\a[27] ), .o1(new_n288));
  nanp02aa1n02x5               g193(.a(\b[26] ), .b(\a[27] ), .o1(new_n289));
  norb02aa1n02x5               g194(.a(new_n289), .b(new_n288), .out0(new_n290));
  xnbna2aa1n03x5               g195(.a(new_n290), .b(new_n284), .c(new_n287), .out0(\s[27] ));
  inv000aa1d42x5               g196(.a(new_n288), .o1(new_n292));
  aobi12aa1n02x7               g197(.a(new_n290), .b(new_n284), .c(new_n287), .out0(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[27] ), .b(\a[28] ), .out0(new_n294));
  nano22aa1n02x4               g199(.a(new_n293), .b(new_n292), .c(new_n294), .out0(new_n295));
  nand23aa1n03x5               g200(.a(new_n238), .b(new_n235), .c(new_n255), .o1(new_n296));
  inv000aa1d42x5               g201(.a(new_n282), .o1(new_n297));
  inv000aa1n02x5               g202(.a(new_n286), .o1(new_n298));
  aoai13aa1n06x5               g203(.a(new_n298), .b(new_n297), .c(new_n296), .d(new_n258), .o1(new_n299));
  aoai13aa1n03x5               g204(.a(new_n290), .b(new_n299), .c(new_n187), .d(new_n283), .o1(new_n300));
  tech160nm_fiaoi012aa1n02p5x5 g205(.a(new_n294), .b(new_n300), .c(new_n292), .o1(new_n301));
  norp02aa1n03x5               g206(.a(new_n301), .b(new_n295), .o1(\s[28] ));
  xnrc02aa1n02x5               g207(.a(\b[28] ), .b(\a[29] ), .out0(new_n303));
  nano22aa1n02x4               g208(.a(new_n294), .b(new_n292), .c(new_n289), .out0(new_n304));
  aoai13aa1n03x5               g209(.a(new_n304), .b(new_n299), .c(new_n187), .d(new_n283), .o1(new_n305));
  oao003aa1n03x5               g210(.a(\a[28] ), .b(\b[27] ), .c(new_n292), .carry(new_n306));
  tech160nm_fiaoi012aa1n02p5x5 g211(.a(new_n303), .b(new_n305), .c(new_n306), .o1(new_n307));
  aobi12aa1n02x7               g212(.a(new_n304), .b(new_n284), .c(new_n287), .out0(new_n308));
  nano22aa1n02x4               g213(.a(new_n308), .b(new_n303), .c(new_n306), .out0(new_n309));
  norp02aa1n03x5               g214(.a(new_n307), .b(new_n309), .o1(\s[29] ));
  xorb03aa1n02x5               g215(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano23aa1n02x4               g216(.a(new_n303), .b(new_n294), .c(new_n289), .d(new_n292), .out0(new_n312));
  aoai13aa1n03x5               g217(.a(new_n312), .b(new_n299), .c(new_n187), .d(new_n283), .o1(new_n313));
  oao003aa1n02x5               g218(.a(\a[29] ), .b(\b[28] ), .c(new_n306), .carry(new_n314));
  xnrc02aa1n02x5               g219(.a(\b[29] ), .b(\a[30] ), .out0(new_n315));
  tech160nm_fiaoi012aa1n02p5x5 g220(.a(new_n315), .b(new_n313), .c(new_n314), .o1(new_n316));
  aobi12aa1n02x7               g221(.a(new_n312), .b(new_n284), .c(new_n287), .out0(new_n317));
  nano22aa1n03x5               g222(.a(new_n317), .b(new_n314), .c(new_n315), .out0(new_n318));
  norp02aa1n03x5               g223(.a(new_n316), .b(new_n318), .o1(\s[30] ));
  norb03aa1n02x5               g224(.a(new_n304), .b(new_n303), .c(new_n315), .out0(new_n320));
  aobi12aa1n02x7               g225(.a(new_n320), .b(new_n284), .c(new_n287), .out0(new_n321));
  oao003aa1n02x5               g226(.a(\a[30] ), .b(\b[29] ), .c(new_n314), .carry(new_n322));
  xnrc02aa1n02x5               g227(.a(\b[30] ), .b(\a[31] ), .out0(new_n323));
  nano22aa1n03x5               g228(.a(new_n321), .b(new_n322), .c(new_n323), .out0(new_n324));
  aoai13aa1n03x5               g229(.a(new_n320), .b(new_n299), .c(new_n187), .d(new_n283), .o1(new_n325));
  tech160nm_fiaoi012aa1n02p5x5 g230(.a(new_n323), .b(new_n325), .c(new_n322), .o1(new_n326));
  norp02aa1n03x5               g231(.a(new_n326), .b(new_n324), .o1(\s[31] ));
  xnrb03aa1n02x5               g232(.a(new_n103), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g233(.a(\a[3] ), .b(\b[2] ), .c(new_n103), .o1(new_n329));
  xorb03aa1n02x5               g234(.a(new_n329), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g235(.a(new_n110), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanp02aa1n02x5               g236(.a(\b[4] ), .b(\a[5] ), .o1(new_n332));
  tech160nm_fioai012aa1n05x5   g237(.a(new_n332), .b(new_n110), .c(new_n122), .o1(new_n333));
  xorb03aa1n02x5               g238(.a(new_n333), .b(\b[5] ), .c(new_n120), .out0(\s[6] ));
  nanp02aa1n02x5               g239(.a(\b[5] ), .b(\a[6] ), .o1(new_n335));
  nanb02aa1n02x5               g240(.a(new_n117), .b(new_n333), .out0(new_n336));
  norb02aa1n02x5               g241(.a(new_n111), .b(new_n114), .out0(new_n337));
  xobna2aa1n03x5               g242(.a(new_n337), .b(new_n336), .c(new_n335), .out0(\s[7] ));
  aoi013aa1n03x5               g243(.a(new_n114), .b(new_n336), .c(new_n337), .d(new_n335), .o1(new_n339));
  xnrb03aa1n02x5               g244(.a(new_n339), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g245(.a(new_n279), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


